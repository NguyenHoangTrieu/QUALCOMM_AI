# dms-ap-uno-q — real Arduino UNO Q -> FDCAN1 -> NXP CAN control test

**Status: CONFIRMED WORKING on real hardware, both directions, 2026-08-15.** Scratch/test copy
(not the canonical repo — see below) of
[`dms-ap`](../../QUALCOMM_AI/DrowsyGuard-EdgeAI-HackTheChallenge2026/dms-ap/README.md)'s control
path, with the one piece that project explicitly left unimplemented now filled in and verified:
**FDCAN1**.

```
you type "2" on this UNO Q  --Bridge-->  sketch.ino  --FDCAN1-->  real CAN bus  --FLEXCAN0-->  vcs-mcxn947
                                                                                                (reacts: L2 alert pattern, speed cap)
you see "simulated_speed=..." <--Bridge--  sketch.ino  <--FDCAN1--  real CAN bus  <--FLEXCAN0--  vcs-mcxn947 (VCS_STATUS, 100 ms)
```

Proof, from the actual bring-up session: `vcs-mcxn947`'s own console (`watch on`) showed
`link=OK` continuously while this sketch was transmitting, and this sketch's own `vcs_status_rx`
counter climbed 1:1 with its `self_test_tx` counter (frames sent == frames received back) — both
directions independently confirmed on two different physical boards' consoles at the same time.

## Hardware setup that is CONFIRMED to work (read this before wiring)

The UNO Q's STM32U585 exposes FDCAN1 as bare 3.3V digital signals on the Arduino header —
**D4 = PA12 = TX, D5 = PA11 = RX** (confirmed from the board's own shipped devicetree, see
`sketch.ino`'s header comment for how that was found). **The UNO Q has no onboard CAN
transceiver** (unlike `vcs-mcxn947`, which has one built in) — you need an external one.

This session used a **TJA1050** module and found, the hard way, that both extremes are wrong:
- **TJA1050 at VCC=3.3V** (out of its 4.5-5.5V datasheet spec): compiles and boots fine, but
  `CAN.write()` starts failing almost immediately (self-test TX counter froze) — the transceiver
  can't drive a strong enough dominant level for the controller's own bit-readback to pass, so
  frames get aborted. Confirmed by watching the TX counter stop incrementing right after this
  change.
- **TJA1050 at VCC=5V with RXD wired straight into D5**: real risk to the STM32U585 GPIO, which is
  not documented as 5V-tolerant on this pin. Not tested this way on purpose.
- **What worked: TJA1050 at VCC=5V (in spec) + a resistor divider on the RXD line only** (TXD
  direction needs nothing extra — 3.3V from the MCU is already a valid HIGH into TJA1050 even at
  5V VCC):
  ```
  TJA1050 RXD ──[1k]──┬── UNO Q D5 (PA11)
                       │
                     [2k]
                       │
                      GND
  ```
  (~3.3V at the divider node from a 5V swing — exact ratio doesn't need to be precise, 1k/2k is a
  convenient off-the-shelf pair.) TXD direction is a straight wire, UNO Q D4 -> TJA1050 TXD.
- `CAN_H`/`CAN_L` from TJA1050 to `vcs-mcxn947`'s J10 header, 120 Ω termination at each physical
  bus end (CAN-002), same as [`dms-sim-mcxn947`](../dms-sim-mcxn947/README.md)'s wiring section.

If you swap in a transceiver with a genuine 3.3V logic-side pin (e.g. one with a separate VIO pin
like the NXP board's own TJA1057, or a native-3.3V part like SN65HVD230), the divider isn't
needed — this session used what was on hand.

## What's new here vs. the canonical `dms-ap`

| Piece | Canonical `dms-ap` | This scratch copy |
|---|---|---|
| Bridge relay (Python <-> sketch) | Implemented, confirmed pattern | **Unchanged**, copied as-is |
| ICD encode/decode (`link/icd.py`, `crc8.py`) | Implemented, tested (61/61 pytest) | **Unchanged**, copied as-is |
| `RouterBridgeTransport` | Implemented | **Unchanged**, copied as-is |
| **FDCAN1** (`sketch.ino`) | **Explicitly NOT implemented** | **New and confirmed working**: uses Arduino's own first-party `CAN` library (`libraries/CAN/CAN.h`, shipped inside the `arduino:zephyr` core package itself — `CAN.begin()`/`CAN.write()`/`CAN.onReceive()`), not raw Zephyr driver calls |
| Camera/inference/fusion pipeline | Implemented (BlazeFace+CNN, 61 tests) | **Not included** — `python/main.py` here is a keyboard-driven alert-level test driver instead, see below |

Nothing in the canonical `dms-ap/` was touched — this is a parallel, minimal app for testing the
CAN control path specifically, per your request to keep new work out of the tracked repo for now.

## How FDCAN1 actually got working — two wrong turns, in order

1. **First attempt: raw Zephyr `zephyr/drivers/can.h`** (`can_send()`/`can_add_rx_filter()`) plus
   hand-written `sketch/boards/*.overlay` and `*.conf` files, based on a real forum post
   ([forum.arduino.cc: "Trying to get fdcan working on the UNO Q"](https://forum.arduino.cc/t/trying-to-get-fdcan-working-on-the-uno-q/1431867))
   where someone got classic CAN working that way. **Compiled fine, failed on real hardware**:
   `device_is_ready()` returned false. Root cause, found by inspecting the *actual* installed
   Arduino core package on this machine (`~/.arduino15/packages/arduino/hardware/zephyr/0.90.0/`):
   the Arduino UNO Q's own shipped board overlay already configures `&fdcan1` (pinctrl, clock
   source, `zephyr,canbus` chosen node — all already correct out of the box), but marks it
   `zephyr,deferred-init;`, meaning the driver's `init()` is deliberately *not* called at boot —
   the app has to call `device_init()` itself, which the first attempt never did. Also: this
   Arduino build pipeline turned out not to use raw west/CMake at all (there's a prebuilt
   `firmwares/zephyr-<variant>.elf/.dts/.config` shipped in the core package, plus a
   `zephyr-sketch-tool` binary that only post-processes an already-linked ELF) — so `boards/*.conf`
   Kconfig fragments were silently never applied either (confirmed: adding `CONFIG_LOG=y` there
   changed nothing about the built binary size).
2. **Second attempt, what's in this repo now**: found `libraries/CAN/CAN.h` bundled inside the
   same installed core package, with real `CANRead`/`CANWrite`/`CANEvent` examples — Arduino's own
   first-party wrapper (`arduino::ZephyrCAN`, global `CAN` object) that calls `device_init()` and
   sets the bitrate internally. Rewrote `sketch.ino` around `CAN.begin(CanBitRate::BR_500k)` /
   `CAN.write(CanMsg(...))` / `CAN.onReceive(...)`. **This is what's confirmed working.** The old
   `sketch/boards/*.overlay`/`*.conf` files were deleted — confirmed unused by this build.

## Build / flash

```bash
arduino-cli core install arduino:zephyr     # once; ~100 MB, not the multi-GB you might expect
cd sketch && arduino-cli compile --fqbn arduino:zephyr:unoq .
arduino-cli upload -p /dev/ttyACMx --fqbn arduino:zephyr:unoq .
```
Or open the whole `dms-ap-uno-q/` folder (`app.yaml` + `sketch/` + `python/`) in App Lab, same as
any Arduino UNO Q app.

Run the Python side either under App Lab (`python/main.py` is what App Lab runs) or standalone
for a bench dry-run first (no `arduino.app_utils` on a dev machine -> `NullTransport`, sends
nothing anywhere, just exercises the keyboard/dashboard logic — confirmed working this way in
this session):
```bash
cd python && python3 main.py
```

## TEMPORARY: bus-level self-test in `sketch.ino`

`loop()` currently also calls `SendSelfTestDmsStatus()` every 100 ms independent of Bridge/Python
— added specifically because this dev session had no way to run `python/main.py` under App Lab on
the UNO Q's own Linux side (no SSH/App Lab access from here) to drive real Bridge traffic, but
still needed to prove FDCAN1 talks to a real `vcs-mcxn947` board. **Remove the whole block marked
`TEMPORARY bus-level self-test` in `sketch.ino`** (the function, its two static counters, and the
call in `loop()`) once `python/main.py` is confirmed driving real alert-level traffic end to end —
at that point Bridge-triggered `on_dms_status()` is the real DMS_STATUS source and the self-test
heartbeat would just be a second, conflicting transmitter on the same CAN ID.

## Using it (once the self-test block above is removed)

Once flashed and running under App Lab with a `vcs-mcxn947` board on the other end of the bus:
- Type `0`-`3` to set the DMS_STATUS alert level sent every 100 ms — the VCS should react exactly
  like it does with `dms-sim-mcxn947` or `sim_uart`: alert pattern, speed cap.
- Type `c` to toggle `flag_calib_done` (needed for the VCS's `DISARMED -> ARMED_IDLE`, VEH-011).
- Every 1 s, a `[dashboard]` line prints `vehicle_state`, `speed_cap_pct`, and
  `simulated_speed_kmh` computed from `VCS_STATUS`'s `duty_left_pct`/`duty_right_pct` — see below
  for exactly what that number does and doesn't mean.
- Type `q` to quit (sends a final L0/`calib_done=False` frame first, DEV-045-style).

## "Simulated vehicle speed" — what it actually is

`vcs-mcxn947` has **no real speed sensor** (see that project's README "What is NOT wired yet" —
motor current/voltage aren't wired either). `VCS_STATUS` carries motor PWM `duty_left_pct`/
`duty_right_pct` (0-100%), not km/h. `python/main.py`'s `MAX_SPEED_KMH = 40.0` is a bench/demo
constant that maps average duty% linearly to a "simulated speed" figure for this dashboard —
`(dutyL + dutyR) / 2 * MAX_SPEED_KMH / 100`. This is **not** a new wire-protocol field — no ICD
change was needed, the VCS already sends duty every 100 ms, this script computes speed from that
on the receiving end only. Change `MAX_SPEED_KMH` (or the whole formula) to whatever fits your
demo; it's an interpretation choice, not a spec value. During the self-test bring-up above, the
vehicle sat in `ESTOP` (physical e-stop sense pin unwired, reads as asserted by design, see
`vcs-mcxn947/README.md`) so duty was 0% the whole time — the CAN link itself was still proven
working independent of that.

## Known gap: VCS-originated EMERGENCY_STOP has no Bridge channel yet

`sketch.ino` receives `EMERGENCY_STOP` off the bus (from the VCS, e.g. its physical e-stop
button) and only `Serial.print()`s it — `ap_rt_transport.py`'s `RouterBridgeTransport` doesn't
register a `Bridge.provide("vcs_estop", ...)` handler, only `vcs_status`/`vcs_event`. Not
invented here since it's outside what you asked for; if you need Python to see VCS-originated
e-stops, that's a matching pair: one `Bridge.call("vcs_estop", ...)` in `sketch.ino`'s
`OnCanReceive()` + one `Bridge.provide("vcs_estop", ...)` + a `poll_estop()`-style method in
`ap_rt_transport.py`.

## What this does NOT do

- No camera, no BlazeFace/CNN, no D1/D2/D3 fusion — `KeyboardAlertSource` stands in for all of
  that. The real pipeline (`dms-ap/app/python/drowsyguard/domains|fusion|inference/`) is
  untouched and can replace it later; only the "what alert_level right now" source needs to
  change, everything downstream (`ControlLoop`, `ap_rt_transport.py`) already matches the real
  app's shape.
- No `DMS_METRICS` transmission — the canonical sketch's `on_dms_metrics` handler is kept (still
  relays to FDCAN1 if Python calls `send_dms_metrics()`), but this test `main.py` never calls it.
- No receive filtering — `CAN.h`'s `ZephyrCAN` wrapper only tracks one standard-ID filter at a
  time (a second `addReceiveFilter()` call replaces the first), and this sketch needs 3 different
  standard IDs, so it accepts every standard frame (the library's own documented no-filter
  behaviour) and dispatches by ID inside `OnCanReceive()` instead.
- Not merged into the canonical `dms-ap/` — copy `sketch.ino`'s `CAN.h`-based additions into
  `../../QUALCOMM_AI/DrowsyGuard-EdgeAI-HackTheChallenge2026/dms-ap/app/sketch/sketch.ino` by hand
  if/when you want this to become the real thing (after removing the TEMPORARY self-test block
  above), and delete the corresponding "FDCAN1: NOT IMPLEMENTED" section header there.
