local active = false
local start_time = 0
local sweep_complete = false

function update()
  -- Configuration
  local CH_TEST    = 1      -- Channel to oscillate: 1=Roll, 2=Pitch, 4=Yaw
  local CH_SWITCH  = 7      -- RC channel used as ON/OFF switch
  local AMP        = 200    -- Amplitude [us] around mid stick
  local FREQ_START = 0.5    -- Starting frequency [Hz]
  local FREQ_END   = 15.0   -- Ending frequency [Hz]
  local SWEEP_TIME = 30.0   -- Time to sweep from start to end [seconds]
  local MID        = 1500   -- Neutral PWM value
  local PERIOD_MS  = 20     -- Update every 20 ms (~50 Hz)
  
  local sw = rc:get_pwm(CH_SWITCH) or 1500
  if sw > 1700 then
    if not active then
      gcs:send_text(0, string.format("Frequency sweep starting: %.1f Hz to %.1f Hz over %.0f sec", FREQ_START, FREQ_END, SWEEP_TIME))
      active = true
      start_time = millis() / 1000.0
      sweep_complete = false
    end
    
    if not sweep_complete then
      local elapsed = (millis() / 1000.0) - start_time
      local freq = FREQ_START + (FREQ_END - FREQ_START) * (elapsed / SWEEP_TIME)
      
      if freq >= FREQ_END then
        freq = FREQ_END
        sweep_complete = true
        gcs:send_text(0, "Frequency sweep complete - stopping")
        rc:clear_override(CH_TEST)
        active = false
      else
        local t = millis() / 1000.0
        local pwm = MID + AMP * math.sin(2 * math.pi * freq * t)
        rc:override(CH_TEST, math.floor(pwm))
      end
    end
  else
    if active then
      gcs:send_text(0, "Sweep stopped by switch")
      active = false
      sweep_complete = false
    end
    rc:clear_override(CH_TEST)
  end
  return update, PERIOD_MS
end

gcs:send_text(0, "Lua Oscillation script loaded")
return update()