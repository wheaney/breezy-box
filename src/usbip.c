#define _POSIX_C_SOURCE 200809L

#include "common.h"
#include "usbip.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>

bool usb_speed_is_super(enum usb_device_speed speed)
{
	return speed == USB_SPEED_SUPER || speed == USB_SPEED_SUPER_PLUS;
}

const char *usb_speed_name(enum usb_device_speed speed)
{
	return usb_speed_is_super(speed) ? "super" : "high";
}

static void set_socket_buffer_best_effort(int fd, int optname, int value)
{
	(void)setsockopt(fd, SOL_SOCKET, optname, &value, sizeof(value));
}

static bool socket_nodelay_enabled(void)
{
	const char *value = getenv("BREEZY_USBIP_TCP_NODELAY");

	return value && value[0] != '\0' && strcmp(value, "0") != 0;
}

int set_socket_options(int fd)
{
	const int one = 1;

	if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != 0)
		return -1;
	if (socket_nodelay_enabled())
		(void)setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
	(void)setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

	/*
	 * USB/IP has no application-level heartbeat, and a DisplayLink stream is
	 * host->box OUT data plus tiny acks: when the host dies ungracefully
	 * (crash, standby, power loss) no FIN arrives and the box blocks forever
	 * in recv_all().  The kernel default keepalive (2h idle + 9*75s probes)
	 * is far too slow to flip the overlay back to "no clients".  Tune it so a
	 * dead peer is detected in ~2s:
	 *   - TCP_KEEPIDLE:  start probing after 1s of idle
	 *   - TCP_KEEPINTVL: 1s between probes
	 *   - TCP_KEEPCNT:   2 unanswered probes
	 * TCP_USER_TIMEOUT bounds the other case: a peer that dies mid-burst while
	 * our RET_SUBMIT acks are still unacknowledged (keepalive only probes an
	 * idle connection).  It also *overrides* keepcnt for the idle case (Linux
	 * closes on keepalive failure only once user_timeout elapses), so it must
	 * track the keepalive budget — keep it ~2s, not larger, or detection would
	 * stretch back out to user_timeout regardless of keepcnt.
	 *
	 * Probes are header-only segments acked by the peer's kernel, so a 1s idle
	 * trigger is cheap even though DisplayLink goes idle between damage bursts.
	 * The tradeoff is false positives on a lossy path: 2 dropped probes (or 2s
	 * of unacked data) in a row will kill a live connection.  Fine on a direct
	 * sub-ms USB-OTG/Ethernet link; revisit if ever exposed over Wi-Fi/WAN.
	 */
	{
		const int keepidle  = 1;
		const int keepintvl = 1;
		const int keepcnt   = 2;
		const int user_timeout_ms = 2000;

		(void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
		(void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
		(void)setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));
		(void)setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT,
				 &user_timeout_ms, sizeof(user_timeout_ms));
	}

	set_socket_buffer_best_effort(fd, SO_RCVBUF, USBIP_STREAM_RCVBUF_SIZE);
	set_socket_buffer_best_effort(fd, SO_SNDBUF, USBIP_STREAM_SNDBUF_SIZE);
	return 0;
}

bool usbip_ack_batch_enabled(void)
{
	const char *value = getenv("BREEZY_USBIP_ACK_BATCH");

	/* Default on; set BREEZY_USBIP_ACK_BATCH=0 to send one ack per submit. */
	return !value || value[0] == '\0' || strcmp(value, "0") != 0;
}

void usbip_set_cork(int fd, int on)
{
	(void)setsockopt(fd, IPPROTO_TCP, TCP_CORK, &on, sizeof(on));
}

void usbip_flush_cork(int fd)
{
	/* Toggling TCP_CORK off then on pushes whatever has accumulated. */
	usbip_set_cork(fd, 0);
	usbip_set_cork(fd, 1);
}

bool usbip_input_pending(int fd)
{
	uint8_t probe;
	ssize_t rc = recv(fd, &probe, sizeof(probe), MSG_PEEK | MSG_DONTWAIT);

	return rc > 0;
}

ssize_t recv_all(int fd, void *buffer, size_t length)
{
	uint8_t *cursor = buffer;
	size_t total = 0u;

	while (total < length) {
		ssize_t rc = recv(fd, cursor + total, length - total, MSG_WAITALL);

		if (rc <= 0)
			return -1;
		total += (size_t)rc;
	}

	return (ssize_t)total;
}

ssize_t send_all(int fd, const void *buffer, size_t length)
{
	const uint8_t *cursor = buffer;
	size_t total = 0u;

	while (total < length) {
		ssize_t rc = send(fd, cursor + total, length - total, 0);

		if (rc <= 0)
			return -1;
		total += (size_t)rc;
	}

	return (ssize_t)total;
}

ssize_t send_iov_all(int fd, const struct iovec *iov, int iovcnt)
{
	struct iovec *local_iov;
	ssize_t total = 0;
	int first = 0;

	if (!iov || iovcnt <= 0)
		return 0;

	local_iov = calloc((size_t)iovcnt, sizeof(*local_iov));
	if (!local_iov)
		return -1;
	memcpy(local_iov, iov, (size_t)iovcnt * sizeof(*local_iov));

	while (first < iovcnt) {
		struct msghdr msg;
		ssize_t rc;
		size_t consumed;

		memset(&msg, 0, sizeof(msg));
		msg.msg_iov = &local_iov[first];
		msg.msg_iovlen = (size_t)(iovcnt - first);

		rc = sendmsg(fd, &msg, 0);
		if (rc <= 0) {
			total = -1;
			break;
		}

		total += rc;
		consumed = (size_t)rc;
		while (first < iovcnt && consumed >= local_iov[first].iov_len) {
			consumed -= local_iov[first].iov_len;
			first++;
		}
		if (first < iovcnt && consumed > 0u) {
			uint8_t *base = local_iov[first].iov_base;

			local_iov[first].iov_base = base + consumed;
			local_iov[first].iov_len -= consumed;
		}
	}

	free(local_iov);
	return total;
}

uint16_t pack_u16(int pack, uint16_t value)
{
	return pack ? htons(value) : ntohs(value);
}

uint32_t pack_u32(int pack, uint32_t value)
{
	return pack ? htonl(value) : ntohl(value);
}

void pack_usbip_usb_device(int pack, struct usbip_usb_device *device)
{
	device->busnum = pack_u32(pack, device->busnum);
	device->devnum = pack_u32(pack, device->devnum);
	device->speed = pack_u32(pack, device->speed);
	device->idVendor = pack_u16(pack, device->idVendor);
	device->idProduct = pack_u16(pack, device->idProduct);
	device->bcdDevice = pack_u16(pack, device->bcdDevice);
}

void pack_op_common(int pack, struct op_common *op)
{
	op->version = pack_u16(pack, op->version);
	op->code = pack_u16(pack, op->code);
	op->status = pack_u32(pack, op->status);
}

int send_op_common_with_payload(int fd,
				uint16_t code,
				uint32_t status,
				const void *payload,
				size_t payload_length)
{
	struct op_common op;
	struct iovec iov[2];
	int iovcnt = 1;
	size_t total_length = sizeof(op);

	memset(&op, 0, sizeof(op));
	op.version = USBIP_VERSION;
	op.code = code;
	op.status = status;
	pack_op_common(1, &op);
	iov[0].iov_base = &op;
	iov[0].iov_len = sizeof(op);
	if (payload && payload_length > 0u) {
		iov[1].iov_base = (void *)payload;
		iov[1].iov_len = payload_length;
		iovcnt = 2;
		total_length += payload_length;
	}
	return send_iov_all(fd, iov, iovcnt) == (ssize_t)total_length ? 0 : -1;
}

int send_op_common(int fd, uint16_t code, uint32_t status)
{
	return send_op_common_with_payload(fd, code, status, NULL, 0u);
}

int recv_op_common(int fd, struct op_common *op)
{
	if (recv_all(fd, op, sizeof(*op)) != (ssize_t)sizeof(*op))
		return -1;
	pack_op_common(0, op);
	if (op->version != USBIP_VERSION) {
		fprintf(stderr,
			"USB/IP version mismatch: remote=0x%04x local=0x%04x\n",
			op->version,
			USBIP_VERSION);
		return -1;
	}
	return 0;
}

void usbip_header_correct_endian(struct usbip_header *header, int pack)
{
	uint32_t command = pack ? header->base.command : 0u;

	header->base.command = pack_u32(pack, header->base.command);
	header->base.seqnum = pack_u32(pack, header->base.seqnum);
	header->base.devid = pack_u32(pack, header->base.devid);
	header->base.direction = pack_u32(pack, header->base.direction);
	header->base.ep = pack_u32(pack, header->base.ep);
	if (!pack)
		command = header->base.command;

	switch (command) {
	case USBIP_CMD_SUBMIT:
		header->u.cmd_submit.transfer_flags = pack_u32(pack, header->u.cmd_submit.transfer_flags);
		header->u.cmd_submit.transfer_buffer_length = (int32_t)pack_u32(pack, (uint32_t)header->u.cmd_submit.transfer_buffer_length);
		header->u.cmd_submit.start_frame = (int32_t)pack_u32(pack, (uint32_t)header->u.cmd_submit.start_frame);
		header->u.cmd_submit.number_of_packets = (int32_t)pack_u32(pack, (uint32_t)header->u.cmd_submit.number_of_packets);
		header->u.cmd_submit.interval = (int32_t)pack_u32(pack, (uint32_t)header->u.cmd_submit.interval);
		break;
	case USBIP_RET_SUBMIT:
		header->u.ret_submit.status = (int32_t)pack_u32(pack, (uint32_t)header->u.ret_submit.status);
		header->u.ret_submit.actual_length = (int32_t)pack_u32(pack, (uint32_t)header->u.ret_submit.actual_length);
		header->u.ret_submit.start_frame = (int32_t)pack_u32(pack, (uint32_t)header->u.ret_submit.start_frame);
		header->u.ret_submit.number_of_packets = (int32_t)pack_u32(pack, (uint32_t)header->u.ret_submit.number_of_packets);
		header->u.ret_submit.error_count = (int32_t)pack_u32(pack, (uint32_t)header->u.ret_submit.error_count);
		break;
	case USBIP_CMD_UNLINK:
		header->u.cmd_unlink.seqnum = pack_u32(pack, header->u.cmd_unlink.seqnum);
		break;
	case USBIP_RET_UNLINK:
		header->u.ret_unlink.status = (int32_t)pack_u32(pack, (uint32_t)header->u.ret_unlink.status);
		break;
	default:
		break;
	}
}
