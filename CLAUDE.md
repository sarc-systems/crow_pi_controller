# CLAUDE.md — monome Crow Multiscale PI Controller

## Project purpose

Implement a reusable controller script for **monome Crow** used as part of a cybernetic live-electronics patch.

Each Crow runs the **same code**. The patching determines whether a particular Crow regulates loudness, brightness, or some other measured quantity.

The controller has:


- **Input 1:** performer-controlled setpoint (normally from Expressive E Touché)
- **Input 2:** measured state from the analog patch (normally an envelope-follower-derived CV)
- **Outputs 1–4:** the same control error represented at four temporal depths:
  1. effectively instantaneous
  2. short-term PI response
  3. medium-term PI response
  4. long-term PI response

The four outputs are intentionally available simultaneously. They are meant to be patched to **different but related actuators** in the analog system, so one regulatory objective can act at several causal depths.

Example: a loudness controller might use the instantaneous output at a final VCA, a short response on daughter-process drive, a medium response on feedback depth, and a long response farther upstream.

This is not intended as a conventional one-output PID controller with four selectable speeds. It is a **multiscale control bank**.

---

## Conceptual model

Let:

    measurement = input[1]
    setpoint    = input[2]
    error       = setpoint - measurement

The sign convention must be easy to reverse globally because some analog destinations may have inverted response.

Output 1 represents the present error with little or no intentional temporal integration.

Outputs 2–4 represent progressively longer histories of the same error.

Conceptually:

    OUT 1 = instantaneous / proportional error
    OUT 2 = short-timescale PI correction
    OUT 3 = medium-timescale PI correction
    OUT 4 = long-timescale PI correction

The meaningful distinction is:

    present -> recent history -> medium history -> long history

Do not treat the four outputs as four independent servos. They share the same measurement, setpoint, and regulatory objective.

---

## Initial design

### Error

Compute a normalized/scaled error:

    e = polarity * (setpoint - measurement)

Provide configurable input scaling/offset if necessary, but keep the first implementation simple.

### Output 1 — "NOW"

Start with a proportional response:

    y1 = Kp_fast * e

Do **not** intentionally slew this output initially.

Crow/ADC/DAC update behavior and the analog envelope follower already impose finite bandwidth. If zippering/chatter becomes a practical problem, add only a very small configurable smoothing constant as an engineering measure, not as one of the musically meaningful temporal scales.

### Outputs 2–4 — temporal PI layers

Maintain three independent integral states:

    I_short  += e * dt
    I_medium += e * dt
    I_long   += e * dt

Each output may include proportional error plus its corresponding integral state:

    y2 = Kp_short  * e + Ki_short  * I_short
    y3 = Kp_medium * e + Ki_medium * I_medium
    y4 = Kp_long   * e + Ki_long   * I_long

However, the implementation should make it easy to test an alternative in which outputs 2–4 are predominantly or entirely integrated/low-passed error. The musical objective matters more than textbook PI orthodoxy.

The three temporal constants should be **widely/logarithmically separated**, not subtle variations of one another.

Starting conceptual targets might be approximately:

    short  ~ 0.1–1 s
    medium ~ 1–10 s
    long   ~ 10–100 s

These are NOT final specifications. They must be exposed as constants near the top of the script and tuned by playing the instrument.

A useful initial ratio is roughly 1:10:100.

---

## Output behavior

All outputs should:

- be continuously updated
- be voltage-limited to a configurable safe range
- have independently configurable output scaling
- optionally have independently configurable polarity
- avoid numerical runaway
- recover gracefully after saturation
- remain deterministic and simple enough to understand while performing

Do not add autonomous modulation, randomness, sequencing, or generative behavior. The Crow is a **regulator/history layer**, not the generative brain of the patch.

---

## Anti-windup

Integral windup is important because analog destinations may saturate or become ineffective over part of their range.

Implement a simple anti-windup strategy.

Preferred first approach:

- clamp each integral state to configurable bounds
- optionally stop integrating farther in the saturated direction when its corresponding output is at its voltage limit

Keep this implementation transparent. Avoid elaborate control-engineering machinery unless testing demonstrates a need.

---

## Deadband

Provide a small configurable error deadband:

    if abs(e) < deadband:
        e_control = 0
    else:
        e_control = e

Default can be zero or very small.

This is experimental. A little deadband may prevent needless hunting around equilibrium; too much will destroy the living/continuous character of the controller.

---

## Update rate

Use a fixed control-loop interval appropriate for Crow and CV control.

The fastest output should feel effectively immediate relative to the musical system, while the other outputs derive their timescale from explicit controller state rather than from a slow global polling rate.

Keep `dt` explicit and use it correctly in integration.

Do not assume ADC/DAC latency constitutes useful slew or integration.

---

## Reset / initialization

On script load:

- initialize all integral states to zero
- initialize outputs safely
- avoid a large startup transient if possible

Provide a simple function or mechanism for resetting the integral histories during development.

If practical, make it easy to add a future "freeze integrators" function, but do not complicate the initial implementation with UI that is not currently required.

---

## Hardware role in the larger patch

There are two Crows in the instrument.

They will initially run identical code:

### Crow A — global loudness

Typical patch:

    analog output/recombined activity
        -> envelope follower
        -> Crow IN 1

    Touché loudness setpoint
        -> Crow IN 2

    Crow OUT 1–4
        -> amplitude-related actuators at different causal/time depths

Likely philosophy:

    OUT 1: final/output-level correction
    OUT 2: nearby process gain/drive
    OUT 3: deeper daughter/feedback parameter
    OUT 4: slow upstream structural influence

Exact destinations are deliberately NOT fixed.

### Crow B — global brightness

Typical patch:

    spectral/brightness detector
        -> Crow IN 1

    Touché brightness setpoint
        -> Crow IN 2

    Crow OUT 1–4
        -> HINGE / spectral actuators at different causal/time depths

Again, destinations remain patch-dependent.

The same code should work for both.

---

## Relationship to the analog patch

The analog system contains autonomous nonlinear feedback processes, including a slipping PLL/APF initiator, an adaptive bifurcation into two daughter processes, matched daughter dynamics, and analog feedback/servo behavior.

Crow should **not** attempt to model or centrally control this system.

Its role is deliberately limited:

1. observe a global variable
2. compare it with a performer-defined setpoint
3. expose corrective pressure at several temporal scales
4. allow the analog patch to decide what those corrections actually do

The performer controls desired global conditions; the analog system determines how it gets there.

This distinction is central to the project.

---

## Important performance principle

Different Crow outputs may be patched simultaneously to different related destinations.

For example:

    instantaneous correction -> final VCA
    short correction         -> process input level
    medium correction        -> feedback amount
    long correction          -> upstream excitation

This allows fast control to correct consequences while slower control gradually acts on causes.

Outputs may also be attenuated, inverted, or cross-patched in the analog domain. An intentionally inverted/adversarial control path is valid and musically desirable. Therefore the Crow script itself should remain neutral and predictable rather than trying to guarantee global stability.

---

## Implementation priorities

In order:

1. Reliable reading of both analog inputs.
2. Correct error calculation.
3. Immediate proportional OUT 1.
4. Three clearly separated temporal PI/history outputs.
5. Output limiting.
6. Integral anti-windup.
7. Easy top-of-file tuning of all important parameters.
8. Clear comments and readable code.
9. Only then consider refinements such as deadband or tiny fast-output smoothing.

Prefer the smallest understandable implementation over abstraction.

---

## Parameters that must be easy to tune

Keep these together near the top of the script:

- control-loop rate / `dt`
- global error polarity
- input scaling/offset if used
- deadband
- Kp for OUT 1
- Kp/Ki or equivalent time constant for OUT 2
- Kp/Ki or equivalent time constant for OUT 3
- Kp/Ki or equivalent time constant for OUT 4
- output voltage min/max
- per-output gain
- per-output polarity
- integral clamps
- optional fast-output smoothing

Do not scatter magic numbers through the program.

---

## Development philosophy

This is an experimental musical control system, not an industrial plant controller.

Do not optimize prematurely for textbook settling time, overshoot, or mathematical elegance. The important tests are:

- Does each temporal layer feel clearly different?
- Are the layers far enough apart in time?
- Does distributing one error signal across related analog functions make behavior more coherent?
- Can slower layers reshape the conditions causing persistent error without destroying faster behavior?
- Does the system remain interesting near instability?
- Can inverted/adversarial patching frustrate regulation in controllable ways?
- Is the relationship between performer setpoint and system response intelligible during improvisation?

The final time constants and gains will be determined empirically.

---

## First milestone

Produce one Crow Lua script implementing:

    IN 1 = measurement
    IN 2 = setpoint

    OUT 1 = effectively instantaneous proportional error
    OUT 2 = short temporal PI/history
    OUT 3 = medium temporal PI/history
    OUT 4 = long temporal PI/history

with conservative voltage limits, anti-windup, centralized configuration constants, and concise comments.

Do not add features beyond this milestone unless required for correct Crow operation.
