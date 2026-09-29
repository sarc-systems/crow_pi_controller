-- pi_controller.lua
-- monome Crow — Multiscale PI Controller
-- One error signal exposed at four temporal depths. See CLAUDE.md for
-- the full design intent; this file is the first milestone.
--
--   IN 1  = setpoint     (performer, e.g. Expressive E Touche)
--   IN 2  = measurement  (analog patch, e.g. envelope follower)
--
--   OUT 1 = NOW     : effectively instantaneous proportional error
--   OUT 2 = SHORT   : short-timescale PI history
--   OUT 3 = MEDIUM  : medium-timescale PI history
--   OUT 4 = LONG    : long-timescale PI history
--
-- These are NOT four independent servos. They share one measurement,
-- one setpoint, and one regulatory objective, patched to different but
-- related actuators so a single goal can act at several causal depths.
--
-- To swap the input mapping in software instead of rewiring, set
-- swap_inputs = true below.

---------------------------------------------------------------------
-- CONFIGURATION  — tune everything here, no magic numbers below.
---------------------------------------------------------------------

-- Timescale ladder for OUT 2-4. The three layers are spaced by a single log
-- base: short : medium : long = 1 : base : base^2. Lower base = layers closer
-- together and the long layer faster; higher base = wider separation.
local TAU_SHORT = 0.5   -- shortest integration timescale, seconds
local TAU_BASE  = 6     -- ratio between consecutive layers

local CFG = {
  ---- control loop ----
  dt = 0.01,             -- loop interval, seconds (100 Hz). Used in integration.

  ---- input handling ----
  swap_inputs = false,   -- true => IN1 = measurement, IN2 = setpoint
  polarity    = 1,       -- global error sign (+1 / -1). Flip to reverse everything.

  meas_gain = 1.0, meas_offset = 0.0,  -- optional measurement conditioning
  set_gain  = 1.0, set_offset  = 0.0,  -- optional setpoint conditioning

  deadband = 0.0,        -- volts; |error| below this is treated as 0 (0 = off)

  ---- OUT 1 : NOW (proportional, deliberately un-slewed) ----
  kp_fast     = 1.0,
  fast_smooth = 0.0,     -- 0 = none. Small value (e.g. 0.2) only to tame zipper
                         -- noise; this is NOT one of the musical timescales.

  ---- OUT 2-4 : temporal PI layers ----
  -- Integration timescale in seconds per layer; Ki is derived as 1/tau.
  -- Spaced by TAU_BASE above (short : medium : long = 1 : base : base^2).
  tau = { short  = TAU_SHORT,
          medium = TAU_SHORT * TAU_BASE,
          long   = TAU_SHORT * TAU_BASE^2 },
  -- Proportional part of each layer. Set a layer to 0 to make that output
  -- purely integrated / low-passed history.
  kp  = { short = 0.5, medium = 0.6, long = 0.7 },

  ---- output stage ----
  out_min = -5.0,        -- conservative safe range (Crow hardware max is +/-10)
  out_max =  5.0,
  gain    = { 1.0, 1.0, 1.0, 1.0 },  -- per-output scaling   (OUT1..OUT4)
  out_pol = { 1,   1,   1,   1   },  -- per-output polarity  (+1 / -1)

  ---- anti-windup ----
  -- Bound on each integral term's CONTRIBUTION in volts (ki*I), NOT the raw
  -- state. This gives every layer equal authority regardless of tau; set it to
  -- the output range so an integrator alone can drive the output full-scale.
  i_authority   = 5.0,
  stop_when_sat = true,  -- also stop integrating further into a saturated output
}

-- Future hook: hold the integral histories in place (see freeze_integrators).
local FREEZE = false

---------------------------------------------------------------------
-- STATE
---------------------------------------------------------------------

local I = { short = 0.0, medium = 0.0, long = 0.0 }  -- integral accumulators
local y1_smoothed = 0.0                              -- OUT1 optional 1-pole state
-- Per-output saturation flags (indices 2..4 used) for anti-windup.
local sat_hi = { false, false, false, false }
local sat_lo = { false, false, false, false }

---------------------------------------------------------------------
-- HELPERS
---------------------------------------------------------------------

local function clamp(x, lo, hi)
  if x < lo then return lo elseif x > hi then return hi else return x end
end

-- Read both jacks, apply mapping + conditioning, return measurement, setpoint.
local function read_inputs()
  local a, b = input[1].volts, input[2].volts   -- a = IN1, b = IN2
  local set, meas
  if CFG.swap_inputs then set, meas = b, a else set, meas = a, b end
  meas = meas * CFG.meas_gain + CFG.meas_offset
  set  = set  * CFG.set_gain  + CFG.set_offset
  return meas, set
end

-- Normalized, polarity-corrected, deadbanded control error.
local function error_signal(meas, set)
  local e = CFG.polarity * (set - meas)
  if math.abs(e) < CFG.deadband then e = 0.0 end
  return e
end

-- OUT 1 : instantaneous proportional error, optionally lightly smoothed.
local function now_output(e)
  local y = CFG.out_pol[1] * CFG.gain[1] * (CFG.kp_fast * e)
  if CFG.fast_smooth > 0 then
    y1_smoothed = y1_smoothed + CFG.fast_smooth * (y - y1_smoothed)
    y = y1_smoothed
  end
  return clamp(y, CFG.out_min, CFG.out_max)
end

-- One temporal PI layer. ch = output index (2/3/4), key = 'short'/'medium'/'long'.
local function pi_layer(ch, key, e)
  local ki  = 1.0 / CFG.tau[key]
  local kp  = CFG.kp[key]
  local pol = CFG.out_pol[ch]
  local g   = CFG.gain[ch]

  -- Integrate, with conditional anti-windup.
  if not FREEZE then
    local dI  = e * CFG.dt
    local eff = pol * g * ki               -- sign of how a rise in I moves output
    local blocked = CFG.stop_when_sat and
        ((sat_hi[ch] and eff * dI > 0) or (sat_lo[ch] and eff * dI < 0))
    if not blocked then
      local i_max = CFG.i_authority / ki   -- bound the contribution ki*I, not I
      I[key] = clamp(I[key] + dI, -i_max, i_max)
    end
  end

  local y    = pol * g * (kp * e + ki * I[key])
  local ylim = clamp(y, CFG.out_min, CFG.out_max)

  -- Record saturation for next step's anti-windup decision.
  sat_hi[ch] = ylim >= CFG.out_max
  sat_lo[ch] = ylim <= CFG.out_min
  return ylim
end

---------------------------------------------------------------------
-- CONTROL LOOP
---------------------------------------------------------------------

local function control_step()
  local meas, set = read_inputs()
  local e = error_signal(meas, set)
  output[1].volts = now_output(e)
  output[2].volts = pi_layer(2, 'short',  e)
  output[3].volts = pi_layer(3, 'medium', e)
  output[4].volts = pi_layer(4, 'long',   e)
end

---------------------------------------------------------------------
-- DEVELOPMENT / PERFORMANCE CONTROLS (callable from the crow REPL)
---------------------------------------------------------------------

-- Zero all integral histories (and OUT1 smoothing) without reloading.
function reset_integrators()
  I.short, I.medium, I.long = 0.0, 0.0, 0.0
  for i = 1, 4 do sat_hi[i], sat_lo[i] = false, false end
  y1_smoothed = 0.0
end

-- Hold / release the integral histories. freeze_integrators() == freeze on.
function freeze_integrators(on)
  FREEZE = (on ~= false)
end

---------------------------------------------------------------------
-- INIT
---------------------------------------------------------------------

function init()
  for i = 1, 4 do
    output[i].slew  = 0      -- no intentional slew; timescales live in the state
    output[i].volts = 0.0    -- safe, transient-free start (integrals start at 0)
  end
  reset_integrators()

  metro[1].event = control_step
  metro[1].time  = CFG.dt
  metro[1]:start()
end
