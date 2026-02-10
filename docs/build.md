# Build Modes

This project supports two build modes via the Makefile: DEBUG and RELEASE.

## Default (Release)

```
make
```

Equivalent to:

```
make BUILD=RELEASE
```

Release defaults:
- `RELEASE_BUILD=1`, `NDEBUG=1`
- `ENABLE_DEBUG_UI=0`
- `ENABLE_SERIAL_LOG=0`
- `ENABLE_PERF_COUNTERS=0` (unless PERF=1)
- `-O3` and LTO if supported (`-flto`)

## Debug

```
make BUILD=DEBUG
```

Debug defaults:
- `DEBUG_BUILD=1`
- `ENABLE_DEBUG_UI=1`
- `ENABLE_SERIAL_LOG=1`
- `ENABLE_PERF_COUNTERS=0` (unless PERF=1)
- `-Og -g3`

## Enable perf counters in release

```
make BUILD=RELEASE PERF=1
```

This enables DWT timing/CPU counters without enabling debug UI overlays or serial logs.

## Helper targets

```
make release
make debug
make check_release
```
