# PreloadToRam Strategy (Release Policy)

This project uses **PreloadToRam** as the release policy. All perform samples are fully loaded into RAM before playback. No SD streaming playback is used in release builds.

## Numeric Guarantees (Baseline)

- **Max voices (polyphony):** 5
- **Max sample length:** 5 seconds @ 48 kHz (240,000 frames)
- **Max simultaneously loaded perform samples:** 1

### RAM Budgets (baseline allocation)

- **Perform sample data:** 960,000 bytes  
  (240,000 frames × 2 channels × 2 bytes)
- **Preview buffers (total):** ~521,984 bytes  
  - preview ring buffer: 32,768 bytes (16,384 frames × 2 bytes)  
  - preview ping/pong buffers: 8,192 bytes (2 × 2,048 frames × 2 bytes)  
  - preview preload buffer: 480,000 bytes (240,000 frames × 2 bytes)  
  - preview read buffer: 1,024 bytes (256 frames × 2 channels × 2 bytes)
- **Record buffers:** 960,000 bytes  
  (reuses the perform sample buffers, mono recorded into both L/R)
- **Waveform cache:** 512 bytes  
  (128 min + 128 max int16 values)

## Buffer Types (Defined and Distinct)

- **Perform sample buffers:** hold the current perform sample in RAM for playback.
- **Preview buffers:** short‑term buffers used for SD preview playback.
- **Record buffers:** use the perform sample buffers as the recording target.
- **Waveform caches:** min/max arrays used for waveform drawing.

## Non‑Goals (Release)

- No SD streaming playback in release builds.
- No blocking I/O in the audio callback.

## Runtime Enforcement

- If a sample load exceeds the RAM budget, the load fails and the previous sample remains.
- If a record exceeds the max frame budget, recording stops at the limit.
- Preview preload is clamped to the max sample frame budget.
- Waveform cache is regenerated on demand and overwrites previous cache data.
