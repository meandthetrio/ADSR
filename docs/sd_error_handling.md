# SD Error Handling Policy

This document defines the SD failure behavior and retry policy for the Daisy Seed sampler.

## States (Single Source of Truth)

- **Ok**: SD present and mounted.
- **NoCard**: SD not detected.
- **MountFailed**: mount attempt failed.
- **FsCorrupt**: filesystem appears corrupt or unsupported.
- **OpenFailed**: file open failed.
- **ReadFailed**: read failed.
- **WriteFailed**: write failed.
- **Timeout**: operation exceeded its deadline.

## User‑Visible Behavior

- **Ok**: Normal operation. All SD operations enabled.
- **NoCard**: UI shows “Insert SD card”. SD ops disabled. Audio continues with already‑loaded RAM samples.
- **MountFailed**: UI shows “Mount failed”. SD ops disabled; “press to retry” allowed.
- **FsCorrupt**: UI shows “FS corrupt”. SD ops disabled; “press to retry” allowed.
- **OpenFailed**: UI shows “File open error”. The specific op fails; other ops may continue.
- **ReadFailed**: UI shows “Read error”. Preview stops; RAM playback continues.
- **WriteFailed**: UI shows “Write error”. Save fails; audio continues.
- **Timeout**: UI shows “SD timeout”. The specific op fails; other ops may continue.

Audio guarantees:
- **Never block audio** on SD errors.
- **Perform playback** keeps running if already loaded in RAM.
- **Preview playback** outputs silence on underrun/fault.

## Retry Policy

- **Max attempts:** 3 per operation.
- **Base backoff:** 200 ms.
- **Max backoff:** 2000 ms.
- **Retryable errors:** MountFailed, ReadFailed, WriteFailed, Timeout.
- **Non‑retryable errors:** FsCorrupt, NoCard (until card present).

## Timeouts

- **Mount:** 1500 ms
- **Scan:** 3000 ms
- **Preview open:** 1500 ms
- **Load:** 8000 ms
- **Save:** 8000 ms
- **Delete:** 2000 ms

## Hard Guarantees

- Never block audio.
- Never infinite loop.
- Never crash on SD removal.
