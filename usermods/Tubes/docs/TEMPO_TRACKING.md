# Experimental microphone tempo tracking

Automatic tempo tracking is an optional local input to the existing Beat channel.
It is disabled at boot and can be enabled on the connected Beat owner with `j1`;
`j0` disables it. Disabling or losing Beat ownership leaves the last accepted clock
running and stops microphone estimates from changing it.

Sound overlays and the audio workshop also start disabled, regardless of persisted
AudioReactive configuration. A `MASTER` build claims Beat-channel ownership during
startup and enables microphone tempo tracking, but its visual overlay remains
disabled until explicitly requested.

## Signal path

AudioReactive continues to own microphone capture and its 512-sample FFT. Most
devices sample at 22,050 Hz, while fixed codecs may require another rate and stereo
slots. It double-buffers each newly mapped 16-band spectrum with a sequence number
and microsecond timestamp before WLED applies visualization gain, decay, smoothing,
and 8-bit scaling. Tubes therefore consumes each raw spectrum once without running
another FFT or mistaking WLED's slow visual decay for new timing:

```text
microphone -> WLED FFT -> raw 16 bands -> positive spectral change
           -> onset history -> tempo correlation -> held Beat clock
```

The onset signal gives WLED bands 0-2 the strongest weight for kick energy and
bands 9-13 a secondary snare/clap path. Broad positive flux contributes 12 percent,
which helps with unusual percussion without letting sustained vocals, pads, or
mastered loudness dominate the clock. A concentrated transient in any single band
also contributes, so a metronome click is not rejected solely because its pitch is
outside the kick and snare groups. Missing FFT frames advance the history with zero
evidence and suppress the unknown cross-gap transition.

The tracker keeps 256 onset frames and searches 70-180 BPM in 0.5 BPM steps every
eight FFT frames. It measures the actual FFT completion cadence from AudioReactive
timestamps, so sample rate, stereo slots, and FFT cost cannot scale the detected
tempo. Candidate scoring uses normalized
autocorrelation, adds weaker two-period evidence, and accumulates a decaying tempo
histogram. A percussive crest gate and a separated winning peak are required before
lock. Several observed onset intervals must also fit the winning beat or half-beat
grid, preventing a short irregular startup burst from producing a confident lock.
The detector then reports 8.8 fixed-point BPM and confidence. A second fixed-size
timing history accepts local maxima from either the kick or snare path, matches them
to the nearest predicted beat, and fits microphone time against beat number across
up to 32 accepted events.

Turning microphone listening off and back on resets the complete estimator state,
including its held tempo, challenger, learned FFT cadence, phase, and confidence.
The next listening session therefore starts with no resistance to a different BPM.

When the tracker first locks, its most recent percussive onset establishes beat 1 through the
same complete BPM-and-beat-frame state consumed from the mesh. The measured
fraction since that onset places the clock at the current instant, while the existing
phrase number remains monotonic. Later onsets within 22 percent of a predicted beat
gently correct the period and phase; off-beat activity is ignored. Each phase update
uses the nearest existing beat, so phrase numbering remains monotonic instead of
restarting. Losing and reacquiring lock establishes a new beat 1.

Every estimate adjusts only the Beat owner's local held clock. The owner publishes
the complete clock to the mesh on beat 1 of each 16-beat measure; followers free-run
between those bounded corrections. Audio accents remain immediate but are rendered
from each pole's own microphone; they no longer send complete Beat-state packets.

All storage is fixed: the onset history and tempo histogram use about 1.9 KB plus
small state arrays. A scan performs at most roughly 100,000 simple correlation
iterations about 5.4 times per second. There is no heap allocation in the audio or
render loop.

## Test boundary

`tools/tempo-tracker-test.cpp` compiles the production estimator as ordinary host
C++. Its deterministic fixtures generate the same 16-band boundary Tubes receives,
including WLED-like fast attack/slow decay, sustained mid-band content, noise,
kick/snare/hat combinations, and dropped frames. The suite proves locks at 90, 120,
128, 135, and 160 BPM, a 120-to-135 BPM transition, eventual unlock after percussion
ends, and that a moving sustained pad does not invent a lock. A 60-second,
snare-only 130 BPM fixture additionally requires 0.15 BPM accuracy and bounded phase
error without any low-frequency kick evidence. A raw floating-point, mid-band 124
BPM metronome must meet the same tempo tolerance, while the irregular onset burst
captured during hardware testing must remain unlocked.

This does not prove microphone gain, room acoustics, or the exact WLED FFT scaling
on a physical pole. `TEMPO_TRACE` exposes lock, BPM, confidence, audio availability,
Beat ownership, the latest signed phase error, and accepted timing-event count so
those remaining variables can be tuned in the field.

## Algorithm sources

The bounded design adapts the spectral-flux, onset-history autocorrelation, tempo
candidate, and decaying-histogram structure described by Michael Krzyzaniak's MIT
licensed [Beat-and-Tempo-Tracking](https://github.com/michaelkrzyzaniak/Beat-and-Tempo-Tracking)
project, inspected at commit `c039090f1af771092d95c3ffc402e557940f7384`. It does
not copy that project's FFT, dynamic allocation, filters, or beat-prediction code.

The held-clock boundary and the separation between tempo evidence and phase guidance
also follow the local Gallery beat listener's design. This firmware implementation
is independent fixed-memory C++ and imports no Gallery or Python code.
