# ESP32 Guitar-to-Bass Sampler

The firmware samples the guitar input on GPIO 34 at 20 kHz using the ESP32's
continuous ADC/DMA driver. It sends centered audio frames through Adam Stark's
chromagram estimator and chord detector, then plays the detected chord root
as the matching bass note in the E2-D#3 range (E guitar chord maps to E2,
F to F2, continuing chromatically through D# to D#3). The serial monitor
reports detected chord names and chroma bins, along with periodic input-level
readings.

Chord estimates use a 2048-point analysis window after the library's 4:1
downsampling, or about 0.41 seconds of input. After starting capture with
`S`, the firmware waits for a full window and two matching estimates before
changing the bass note. The current output is a synthesized bass tone; pad
chord playback is not yet implemented.

The chord detector always selects its closest chord profile and does not
report a confidence score or a "no chord" result. Its chroma templates start
at C3, so lower guitar notes rely on their upper harmonics to identify their
pitch classes. Use sustained, clearly voiced chords for the most reliable
results.

## Build

Build with PlatformIO for the `esp32dev` environment. The chromagram uses the
bundled KissFFT backend; `platformio.ini` enables it with `USE_KISS_FFT`.

## Licensing

The firmware is distributed under the GNU General Public License, version 3.
The upstream chromagram/chord detector and complete GPL text are in
[`lib/ChordDetector/LICENSE.txt`](lib/ChordDetector/LICENSE.txt). KissFFT is
distributed under its permissive license in
[`lib/ChordDetector/KISSFFT-COPYING`](lib/ChordDetector/KISSFFT-COPYING).
The vendored library is based on upstream commit
`b2b9e756d6d56098fcb9605bc776155314c1fe08`; local changes reduce its FFT
window to 2048 samples, preserve the anti-alias filter state across frames,
and reuse frame-processing buffers to avoid repeated heap allocation.
