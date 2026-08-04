# Coil Winder Firmware

English | [Русский](README.ru.md) | [Українська](README.uk.md)

Arduino firmware for a two-axis coil winding machine: a stepper-driven **main shaft** (spindle) that rotates the bobbin, and a stepper-driven **wire guide** (traverse carriage on a lead screw) that lays the wire evenly across the coil width, mechanically synchronized to the main shaft rotation. Control and status are handled through a 16x2 character LCD and a rotary encoder with push-button.

## Demo

[![Coil winder demo](https://img.youtube.com/vi/PlyOI3DIClg/0.jpg)](https://youtube.com/shorts/PlyOI3DIClg)

More videos on [@viktoruzhgorod on YouTube](https://www.youtube.com/@viktoruzhgorod).

## Hardware

| Function | Driver / Component | Pins (Arduino) |
|---|---|---|
| Main shaft stepper | DRV8825, **16x microstepping** | STEP `9`, DIR `8`, ENABLE `10` |
| Wire guide stepper | DRV8825, **8x microstepping** | STEP `11`, DIR `6`, ENABLE `5` |
| Rotary encoder | quadrature + push button | A `3`, B `4`, Button `2` |
| Wire guide home switch | limit switch (input pullup) | `7` |
| LCD | 16x2 character, 4-bit mode | RS `A0`, EN `A1`, D4-D7 `A2-A5` |

Motor: 200 full steps/rev (1.8°). Wire guide lead screw: 1mm pitch.

- Main shaft resolution: 200 × 16 = **3200 microsteps/revolution**
- Wire guide resolution: 200 × 8 = 1600 microsteps/rev → **1600 steps/mm** of carriage travel (0.625 µm/step)

## Core features

### Bresenham-synchronized wire guide
Instead of running the wire guide off its own timer, every main-shaft microstep increments an accumulator by a fixed ratio (`wgRatio`, derived from wire diameter). Whenever the accumulator overflows a full main-shaft revolution's worth of steps, the wire guide is pulsed forward by one step. This keeps the wire pitch locked to the spindle rotation regardless of speed, with no drift from independent timers. The accumulator drains with a `while` loop (not a single `if`) so it always stays exactly in sync even if the required ratio ever exceeds one wire-guide step per main step.

### Automatic layer reversal
The firmware tracks the wire guide's position within the current layer (`currentLayerPosition`). When it reaches the coil length (`coilLengthMM`), the traverse direction is reversed and the layer counter increments — the coil is built up in successive back-and-forth layers automatically, with no operator intervention needed in AUTO/SEMI-AUTO modes.

### Configurable wind direction (L→R / R→L)
By default the wire guide's first pass moves **left-to-right**, away from the home limit switch. It can instead be set to start **right-to-left**, moving toward the switch. In R→L mode, the configured zero offset (`zeroOffsetMM`) is interpreted as the distance from the switch to the *far* starting point, and the coil length is automatically clamped so it can never exceed that offset — this prevents the carriage from being commanded past the physical limit switch.

### Smoothstep speed ramping
Motor start/stop is ramped over ~1 second using a smoothstep curve (`3x²-2x³`) rather than an instant jump to target speed, reducing mechanical shock and skipped steps at high speed.

### Live speed adjustment while running
In any run mode, turning the encoder adjusts `targetRPS` in ±0.1 RPS steps, live, without stopping the motor. Turning it enough to reach 0 performs a controlled hard stop; continuing past 0 reverses the main shaft direction.

### Auto-stop on target turns
In SEMI-AUTO and AUTO modes, the winder automatically ramps down and stops once the spindle reaches the configured target turn count.

### Manual reverse floor
In MANUAL mode, reversing the spindle automatically stops the motor once the turn counter returns to 0, preventing the count from going negative.

### Homing & zero-set sequence
On power-up the firmware requires the wire guide to be homed against its physical limit switch, then lets the operator jog to and confirm a "zero offset" — the actual coil-winding start position relative to that switch (0.1mm resolution, 0–200.0mm).

## System modes

| Mode | Purpose |
|---|---|
| `HOMING_MODE` | Drives the wire guide to the limit switch to establish a physical zero (runs once at boot, on button press). |
| `ZERO_SET_MODE` | Operator dials in the offset from the physical switch to the logical winding-start position. |
| `MENU_MODE` | Encoder scrolls between MANUAL / SEMI-AUTO / AUTOMATIC / SETUP; button selects. |
| `MANUAL_MODE` | Free-running spindle, forward or reverse, speed adjustable live, no auto-stop on turns (stops at 0 turns in reverse). |
| `SEMI_AUTO_MODE` | Winds one layer, then automatically stops the motor at every layer boundary so the operator can install inter-layer insulation; button press resumes the next layer. Also auto-stops for good at the target turn count. |
| `AUTO_MODE` | Fully automatic: winds continuously through all layers up to the target turn count, then stops. |
| `SETUP_MODE` | 4-parameter configuration screen (see below). |

## Setup parameters

Reached via the menu's SETUP option, cycled with the encoder button:

1. **Wire diameter** — 0.05–1.00 mm, 0.01 mm steps. Capped at 1.00 mm; thicker wire is expected to be wound manually. Determines the wire-guide/spindle gearing ratio.
2. **Coil length** — 1–500 mm, 1 mm steps. Width of the traverse per layer. Automatically capped at the zero offset when wind direction is R→L.
3. **Target turns** — 1–9999, whole turns. Also used to estimate and display the number of layers required (`turns / (coilLength / wireDiameter)`, rounded up).
4. **Wind direction** — left-to-right (default) or right-to-left, shown on the LCD with arrow glyphs.

## Speed

- Adjustable range: **0 to ±3.0 RPS** (rotations per second), in 0.1 RPS steps via the encoder while running.
- At 16x microstepping this corresponds to up to 9600 microsteps/sec on the main shaft — comfortably within the DRV8825's electrical timing limits and the firmware's 10µs timer resolution.
- Sign of `targetRPS` sets spindle direction; magnitude sets speed.

## Typical use cases

**Winding a simple single-layer coil (AUTO mode)**
1. Power on → home the wire guide against the limit switch (button press).
2. Jog to the desired start offset from the switch, confirm.
3. In SETUP, dial in wire diameter, coil length, and target turns.
4. From the menu, select AUTOMATIC and press the button to start — the winder ramps up, winds the full turn count across as many layers as needed, reversing direction at each layer boundary automatically, and stops itself at the target turn count.

**Manual winding / touch-up work**
- Select MANUAL for direct, un-timed control: start/stop with the button, adjust speed live with the encoder, reverse by dialing speed through 0. Useful for hand-guiding unusual wire or finishing partial coils.

**Layer-by-layer supervised winding (SEMI-AUTO)** — likely the most commonly used mode

SEMI-AUTO behaves exactly like AUTO (automatic turn-counting, layer traversal, and final auto-stop at the target turn count), with one key difference: the motor is automatically stopped every time a layer boundary is reached (`layerCount` increments and the wire guide reverses direction), instead of continuing straight into the next layer.

This pause exists specifically so the operator can install **inter-layer insulation** — tape, paper, or film — between windings before the next layer starts, which is standard practice for transformer/inductor coils to maintain insulation and even layering. Once the insulation is placed, pressing the button resumes winding the next layer from exactly where it left off (turn count and wire-guide position are preserved, nothing is reset). This repeats at every layer boundary until the target turn count is reached, at which point the winder stops for good, the same as AUTO mode.

**Right-to-left coils**
- For bobbins/fixtures where the wire needs to be dressed starting from the far end and finishing near the limit switch, set wind direction to R→L in SETUP. The firmware then treats the dialed zero offset as the far starting point and prevents a coil length that would drive the carriage past the switch.

## Firmware internals of note

- Main-shaft stepping and wire-guide sync both happen inside a single `TIMER1` compare-match ISR running every 10µs; the wire-guide step pulse is emitted inline (blocking ~3µs) whenever the Bresenham accumulator overflows, so it stays phase-locked to the spindle without any additional timer or polling loop.
- `stepCount` (the authoritative turn counter) is only ever read by the main loop under a brief `cli()/sei()` critical section, since it's a multi-byte value written from inside the ISR.
- Encoder and button inputs are handled by pin-change interrupts with simple debounce (150 ms for the button).
