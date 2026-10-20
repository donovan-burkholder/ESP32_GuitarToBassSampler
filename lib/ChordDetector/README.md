# Chord Detector and Chromagram

Vendored from [adamstark/Chord-Detector-and-Chromagram](https://github.com/adamstark/Chord-Detector-and-Chromagram),
upstream commit `b2b9e756d6d56098fcb9605bc776155314c1fe08`.

`Chromagram.cpp` uses the bundled KissFFT implementation selected by
`USE_KISS_FFT`. The upstream 8192-sample FFT window is reduced to 2048 samples
to limit ESP32 heap use and provide quicker chord updates. Frame-processing
vectors are reused instead of allocated on every call.

See `LICENSE.txt` for the upstream GPL-3.0 license and `KISSFFT-COPYING` for
KissFFT's license.
