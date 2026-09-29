# Crow Multiscale PI Controller

A single [monome Crow](https://monome.org/docs/crow/) Lua script (`pi_controller.lua`)
that regulates one measured quantity against a performer-controlled setpoint and
exposes the resulting control error at **four temporal depths** simultaneously.
It is a *multiscale control bank*, not a conventional single-output PID: the four
outputs are meant to be patched to different-but-related actuators so one
regulatory objective can act at several causal depths at once.

Both Crows in the instrument run this identical code; the patching decides what
each one regulates (e.g. loudness vs. brightness). See `CLAUDE.md` for the full
design rationale.

## Signal flow

```
IN 1  = setpoint     (performer, e.g. Expressive E Touché)
IN 2  = measurement  (analog patch, e.g. an envelope-follower CV)

error = polarity * (setpoint - measurement)

OUT 1 = NOW     : effectively instantaneous proportional error
OUT 2 = SHORT   : short-timescale history
OUT 3 = MEDIUM  : medium-timescale history
OUT 4 = LONG    : long-timescale history
```

(To swap the input jacks in software, set `swap_inputs = true`.)

## Control law

- **OUT 1** is proportional to the present error (`kp_fast`), deliberately un-slewed.
- **OUT 2–4** are three independent temporal layers, `y = kp*e + ki*I` with
  `ki = 1/tau`. By default (`leaky = true`) each layer is a **leaky integrator**
  — a first-order low-pass of the error over its own `tau`, so when the error
  returns to zero each output relaxes back to rest over that timescale rather
  than holding a wound-up value. Set `leaky = false` for pure PI integration.
- The three timescales come from one log-base knob: `short : medium : long =
  1 : TAU_BASE : TAU_BASE²`.
- **Anti-windup:** each layer's integral *contribution* (`ki*I`) is clamped to
  `±i_authority` volts (equal authority regardless of `tau`), and integration
  stops pushing further into a saturated output when `stop_when_sat` is set.
- All outputs are clamped to a safe voltage range with per-output gain and polarity.

## Tuning

Everything lives in the `CFG` table (plus `TAU_SHORT`/`TAU_BASE`) at the top of
`pi_controller.lua`. Current working values (not final):

| Parameter | Value | Meaning |
|---|---|---|
| `dt` | `0.01` | control-loop interval (100 Hz) |
| `polarity` | `1` | global error sign (`+1`/`-1`) |
| `deadband` | `0.0` | error deadband, volts |
| `kp_fast` | `1.0` | OUT 1 proportional gain |
| `fast_smooth` | `0.0` | optional OUT 1 smoothing (0 = off) |
| `TAU_SHORT` | `0.5` | shortest timescale, seconds |
| `TAU_BASE` | `6` | ratio between layers → `{0.5, 3, 18} s` |
| `kp` | `{0.5, 0.6, 0.7}` | per-layer proportional part |
| `leaky` | `true` | leaky (low-pass) vs. pure integration |
| `out_min`/`out_max` | `-5.0` / `5.0` | output voltage limits |
| `gain` / `out_pol` | `{1,1,1,1}` | per-output scale / polarity |
| `i_authority` | `5.0` | max integral contribution, volts |
| `stop_when_sat` | `true` | halt integration into a saturated output |

Lower `TAU_BASE` for tighter, faster layers; raise it for wider separation.

## Flashing to Crow

Uses monome's [`druid`](https://github.com/monome/druid) over USB serial:

```sh
pip install monome-druid       # one-time
druid run    pi_controller.lua # run once in RAM (test, non-persistent)
druid upload pi_controller.lua # write to Crow's user-script flash (persists on boot)
```

The Crow performs standalone from flash — no computer is attached during use.
`druid` is a development/tuning tool only.

Development helpers callable from the `druid repl`: `reset_integrators()` and
`freeze_integrators(on)`.
