# Vendored dependencies

## mongoose

Single-file HTTP/WebSocket library used by `breezy_web`.

These files are **not committed** (see `.gitignore`).  Fetch them with:

```sh
make deps
```

This downloads `mongoose.h` and `mongoose.c` from the mongoose master branch
into this directory.  After that, `make` / `make breezy_web` will compile them
normally.  Re-run `make deps` to update to a newer snapshot.
