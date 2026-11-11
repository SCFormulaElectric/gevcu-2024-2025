# Bamocar / VCU: open work

Remaining work on the `bamocar-speed-limiter` branch, most important first.

## 1. Bamocar drops every other back-to-back frame (fix first)

Each second, `applySpeedLimiter()` sends the Bamocar (ID 0x201) 5 register writes and then 10 read requests, all back to back. The `BAMOCAR SETTINGS (read back)` report shows every second request with no reply:

```
N-Lim OK | M R-Acc no reply | M R-Dec OK | N R-Acc no reply | N R-Dec OK | speed Kp no reply | speed Ti OK
current ramp no reply | I max eff OK | I max pk no reply | I con eff OK
```

This looks like the Bamocar missing frames that arrive too close together. That's inferred from the alternating pattern, not confirmed. The writes are probably affected the same way:

- **0x35 (M R-Acc / N R-Acc):** 2nd frame, probably dropped. This means `MRACC` likely isn't reaching the inverter.
- **0x2C (speed Kp):** 4th frame, probably dropped.

**Fix:** queue the Bamocar writes and read requests, and send one per 10 ms tick instead of all at once. The torque command (0x90) shares ID 0x201, so make sure a queued frame never goes out immediately before or after it in the same tick.

**Done when:** every item in the report reads `OK` and `M R-Acc 25 ms OK` shows up after `MRACC=25`.

`BmsClearer` also sends bursts to 0x201: 0x20 and 0xA0 every second, plus 0x25/0x4D/0xC4/0xC5 at startup. Its I max pk (0xC4) request has never been answered either, so it should go through the same queue.

## 2. Find what limits current

Peak battery current so far is 130 (log -08, 2026-11-07). The VCU doesn't cap current: full pedal is 100% of the Bamocar's I max pk. Read and log the Bamocar's own derating flags to see which limit is active:

- **0x40 status bits:** 21 at current limit, 5 reduced to continuous, 22 speed derating, 24 IGBT temperature, 25 below 10 Hz, 26 motor temperature, 28 T-peak warning.
- **Registers:** 0x48 (current limit in use), 0x20 (actual current), 0xF0 (T-peak), 0x3C (I-red-N), 0x46 (I lim dig).

Broadcast these on CAN so MoTeC logs them next to `bms_current`.

## 3. Dropouts with fault_code 29

In every MoTeC log, all channels drop to -1 at the same moment, and `fault_code` reads 29 right after. This happens when HV comes up and also mid-run. Log -07, run with the LoRa telemetry board connected, had 13 of them. `BmsClearer` can only produce fault numbers 1–28, so 29 isn't a real BMS fault. Check the 12 V / 5 V supply and the CAN wiring. Details are in the 2026-11-07 log notes.

## 4. Smaller fixes

- **`BmsClearer` mislabel:** it prints register 0xA0 as "BAMO STATUS", but 0xA0 is M out (actual torque current). The status register is 0x40. The print is commented out right now, but the label is still wrong.
- **`RequestDCMsg` leftovers:** it's set up in `setup()` but no longer sent. Delete it. Its first two bytes were swapped anyway (`EB 3D` instead of `3D EB`).
- **Unused variables:** `secondhalf` and `firsthalf` in `BamocarMotorController::handleTick()` are unused since the `TORQUE SENT` print was commented out.
- **Unused settings:** these do nothing on the Bamocar and could be hidden or noted in the console help: `TORQSLEW`, `RPMSLEW`, `REVLIM`, `TAPERHI/LO`, `ENABLEIN`, `REVIN`. `TORQ` only feeds CAN message 0x473.
- **`Logger::console` buffer:** it formats into a 200-byte buffer with `vsprintf`, so any message longer than that overflows the stack. Switch it to `vsnprintf`.
- **`realrpm` in MoTeC:** it peaks at only 40–58 while the car pulls 70–130 A. Check the logger's decode of CAN 0x474 (signed 16-bit, big-endian, rpm).

## 5. Test on the car

- **10 ms timing:** the torque command and pedal reads now run every 10 ms instead of 40 ms. This hasn't been driven yet. Check throttle feel and that the CAN bus stays healthy.
- **Torque ramp:** after #1 is fixed, compare `MRACC=25` and `MRACC=50` against the stock 100 ms for feel, drivetrain shock and wheelspin.
