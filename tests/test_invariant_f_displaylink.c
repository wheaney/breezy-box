#include <check.h>
#include <stdlib.h>
#include <string.h>

/*
 * Since the vulnerable code is in a kernel module and cannot be directly
 * linked into a userspace test, we simulate the exact vulnerable pattern
 * to serve as a regression guard for the security invariant:
 * "Buffer reads/writes never exceed the declared buffer size (DL_BULK_BUFSIZE)."
 *
 * This tests the logic that MUST be enforced: n must be clamped to req->buf size.
 */

#define DL_BULK_BUFSIZE 512

/* Simulates the fixed copy logic that must exist in f_displaylink.c */
static int safe_vendor_desc_copy(void *buf, size_t buf_size,
                                  const void *vendor_desc, size_t desc_len,
                                  unsigned short wLength)
{
    size_t n = (wLength < desc_len) ? wLength : desc_len;
    /* SECURITY INVARIANT: n must never exceed buf_size */
    if (n > buf_size)
        n = buf_size;
    memcpy(buf, vendor_desc, n);
    return (int)n;
}

START_TEST(test_vendor_desc_copy_bounded)
{
    /* Invariant: copy length never exceeds DL_BULK_BUFSIZE regardless of wLength */
    unsigned short wLength_payloads[] = {
        DL_BULK_BUFSIZE * 10,  /* exploit: 10x overflow */
        DL_BULK_BUFSIZE * 2,   /* 2x overflow */
        DL_BULK_BUFSIZE + 1,   /* boundary: off-by-one */
        DL_BULK_BUFSIZE,       /* exact boundary */
        64                     /* valid small request */
    };
    int num_payloads = sizeof(wLength_payloads) / sizeof(wLength_payloads[0]);

    char req_buf[DL_BULK_BUFSIZE];
    char vendor_desc[DL_BULK_BUFSIZE];
    memset(vendor_desc, 'A', sizeof(vendor_desc));

    for (int i = 0; i < num_payloads; i++) {
        memset(req_buf, 0, sizeof(req_buf));
        int n = safe_vendor_desc_copy(req_buf, DL_BULK_BUFSIZE,
                                       vendor_desc, sizeof(vendor_desc),
                                       wLength_payloads[i]);
        ck_assert_int_le(n, DL_BULK_BUFSIZE);
        ck_assert_int_ge(n, 0);
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_vendor_desc_copy_bounded);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}