# Motor bring-up (STS3215 + Waveshare ESP32)

## 0. Host setup (Ubuntu)
```bash
pip install platformio
sudo usermod -aG dialout $USER     # log out/in once
ls /dev/ttyUSB*                     # board should appear (CP210x)
```
If `/dev/ttyUSB0` appears then vanishes: `sudo apt remove brltty`.

## 1. Flash
```bash
cd firmware
pio run -t upload
pio device monitor
```
Overwrites the Waveshare stock web-UI firmware. Expect boot line + `found N: ...`.

## 2. Assign IDs — ONE SERVO AT A TIME
New STS3215s all ship as **ID 1**. With several on the bus they answer simultaneously and corrupt replies.

For each servo:
1. Power off. Connect **only** that servo.
2. Power on, `scan` → `found 1: 1`
3. `setid 1 <new>` → `ok: 1 -> <new>`
4. Label the servo with its ID. Next.

The servo destined for ID 1 needs no change. Do it last.

## 3. Verify the full bus
Daisy-chain all, `scan` → `found 3: 1 2 3`.

## 4. Unloaded motion check (no tendons attached)
```
read 1
pos 1 2048
pos 1 2500 500 30
pos 1 2048
stream all 50
stream off
stop
```
- `pos` raw units: 4096 counts/rev (≈0.088°/count), 2048 = center.
- `load` is signed and roughly ‰ of max torque; this is the tendon-tension proxy. Record the no-load baseline now.
- `volt` in decivolts. Check it matches the PSU under motion.

## 5. Winding + calibration (after winches are mounted)
Don't use the console's `zero` (EEPROM) for this. Follow `docs/software.md` → *Tendon setup*:
`rcm rest` (hold at rest pose) → wind with slight slack → dashboard *Calibrate*.

## Known limits for winch use
Position mode covers **one revolution** (0–4095). If a tendon needs more spool travel than one rev of the winch, we need multi-turn (step) mode. Decide once spool diameter and tendon stroke are known: stroke ≤ π·d_spool fits position mode.
