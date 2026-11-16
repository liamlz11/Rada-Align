# Demo rig and parts (task 3)

STATUS: PARTIAL. We do not yet know which parts the hackathon will give us or
the demo distance (2 m is the working value). So this file is written the other
way round: what any part must achieve, a rig that works with several kinds of
parts, and a table to decide in minutes once the parts are on the table.
Sections 1 and 2 must be redone with the real parts; nothing here has been
copied into config.yaml.

**Final design (2026-11-16):** sections 1 and 2 (X/Z, steppers, matched
camera/beacon offsets) are SUPERSEDED for the new rig and kept for history.
The new parts list is section 4 below and docs/FINAL_DESIGN.md section 9. The
datasheet facts in section 3 (rangefinder range, inclinometer offset) still hold.

All resolution numbers come from `python scripts/rig_resolution.py` (default
2 m and 12 m; pass other distances as arguments). The parts in that script are
generic candidates, each an ASSUMPTION about a typical part.

## 1. Rig

### What the rig must do, whatever it is built from

| Need | 2 m demo | 12 m field | Why |
| --- | --- | --- | --- |
| Tolerance as a horn offset | 1.75 mm | 10.47 mm | 0.05 deg |
| X/Z travel for a +/-1 deg setup error | +/-35 mm | +/-209 mm | d * tan(1 deg) |
| Step size on every axis | <= 0.005 deg (OK), <= 0.01 deg (marginal) | same | must be clearly finer than the 0.01 deg deadband |
| Camera/beacon offset mismatch known to | 0.3 mm (0.0086 deg) | 2 mm (0.0095 deg) | REVIEW W3 |
| Rigid link camera-beacon-reference | no visible flex by hand | same | any flex is an invisible bias |

### Rules that do not depend on the parts

1. **Matched offsets** (rig rule in config.yaml). On each head the camera and the
   beacon sit at mirror positions: if the radar-side camera is 100 mm to the
   right of the radar axis, the horn-side beacon is 100 mm to the horn's left of
   the horn center, at the same height, and the same the other way round. Seen
   from above the two cameras face the two beacons along parallel lines.
2. **One plate per head.** Camera, beacon and the reference feature are fixed to
   the same stiff plate; nothing is clamped to a tripod leg or a cable.
3. **Radar-side head carries no motors.** It only needs a repeatable seat on the
   radar mock-up: three contact points, not a flat clamp (REVIEW W1).
4. **Pan/tilt pivot as close to the horn aperture center as the parts allow.**
   Every mm of lever arm couples pointing into position; the loop handles it,
   but it costs iterations.
5. **Small LED aperture at short range.** A 1 mm pinhole over the beacon for the
   table (REVIEW W4).

### Text sketch (top view, not to scale; lengths are a proposal, not measured)

```
        RADAR SIDE (fixed)                          HORN SIDE (motorized)

   beacon B_r   radar axis    camera C_r       camera C_h   horn axis   beacon B_h
       o------------+------------[ ]               [ ]----------+-----------o
       |<- 100 mm ->|<- 100 mm ->|                 |<- 100 mm ->|<- 100 mm->|
                    |                                           |
              radar mock-up                               horn mock-up
              (flat plate with                      on Pan/Tilt, on Z, on X
               3-point seat)

       C_r looks at B_h  ......................................  (parallel lines,
       C_h looks at B_r  ......................................   200 mm apart)

                    |<--------------- d = 2 m (demo) -------------->|
```

Side view of the horn mount, bottom to top: base plate -> X slide (across the
line of sight) -> Z lift -> Pan (vertical axis) -> Tilt (horizontal axis) ->
plate with horn mock-up, camera and beacon.

### Which axes to motorize if we cannot build all four by Monday

Priority order, from the error that matters most:

1. **X and Z** correct the radar error, which is the requirement (R3, R6). At
   2 m they only need +/-35 mm of travel.
2. **Pan and Tilt** correct the horn error. If the horn's beamwidth is tens of
   degrees this is far less critical (REVIEW W3, open question 5), so with only
   two motors: motorize X and Z, set Pan/Tilt by hand with fine screws, and show
   the horn error live on the dashboard for the operator.
3. With zero suitable motors: manual micrometer stages and "dry-run" mode (task
   8): the system measures and tells the operator how far to turn each screw.
   That still demonstrates R1, R2, R3, R5, R8 and R10, not R4 and R11.

## 2. Angular resolution: decide once the parts are known

Output of `python scripts/rig_resolution.py`.

### Pan / tilt (distance does not matter)

| Drive | deg per step | Verdict |
| --- | --- | --- |
| Hobby servo (CHALLENGE.md: ~1 deg) | 1.0 | too coarse |
| Stepper direct, full step | 1.8 | too coarse |
| Stepper direct, 1/16 microstep | 0.1125 | too coarse |
| Stepper direct, 1/256 microstep (nominal, not repeatable) | 0.00703 | marginal |
| 28BYJ-48 geared stepper, 4096 half-steps/rev [unverified] | 0.0879 | too coarse |
| Stepper + 5:1 belt, 1/16 | 0.0225 | too coarse |
| Stepper + 100:1 worm, 1/16 | 0.00113 | OK |
| Stepper + 90:1 worm stage, 1/8 | 0.0025 | OK |
| Tangent arm: 0.5 mm/rev screw on a 100 mm arm, full step | 0.00143 | OK |
| Tangent arm: 0.8 mm/rev screw on a 150 mm arm, full step | 0.00153 | OK |
| Tangent arm: 8 mm/rev screw on a 150 mm arm, 1/16 | 0.00095 | OK |

The **tangent arm** is the fallback if there is no worm gear: a stepper turns an
ordinary screw that pushes the end of a lever, with a spring holding the lever
against the screw (the spring also removes backlash). +/-2 deg needs only
+/-3.5 mm of screw travel on a 100 mm arm, +/-5.2 mm on 150 mm. A direct-drive
stepper or a hobby servo cannot do the job at any microstepping.

### X / Z

| Drive | mm per step | deg at 2 m | deg at 12 m | Verdict at 2 m |
| --- | --- | --- | --- | --- |
| Lead screw 8 mm/rev, full step | 0.04 | 0.00115 | 0.00019 | OK |
| Lead screw 8 mm/rev, 1/16 | 0.0025 | 0.00007 | 0.00001 | OK |
| GT2 belt, 20-tooth pulley, full step | 0.2 | 0.00573 | 0.00095 | marginal |
| GT2 belt, 20-tooth pulley, 1/16 | 0.0125 | 0.00036 | 0.00006 | OK |
| Hand micrometer stage, 10 um | 0.01 | 0.00029 | 0.00005 | OK (manual) |

Linear axes are easy: almost any stepper-driven slide is fine enough. The risk
is play and sag, not step size.

### Cameras

| Camera | deg per px | FOV (deg) | mm per px at 2 m | Centroid needed for 0.005 deg |
| --- | --- | --- | --- | --- |
| 12 MP, 1.55 um pixels + 25 mm lens | 0.0036 | 14.3 x 10.8 | 0.124 | 1.41 px |
| same sensor + 16 mm lens | 0.0056 | 22.2 x 16.8 | 0.194 | 0.90 px |
| same sensor + 6 mm lens | 0.0148 | 55.3 x 42.9 | 0.517 | 0.34 px |
| 1080p webcam, 70 deg horizontal FOV | 0.0418 | 70.0 x 43.0 | 1.459 | 0.12 px |
| 720p webcam, 60 deg horizontal FOV | 0.0517 | 60.0 x 36.0 | 1.804 | 0.10 px |

An ordinary webcam can work, but only with 0.1 px centroiding, which needs a
clean, unsaturated, well-sampled spot and fixed exposure. Task 4's
`measure_stability.py` gives the real number for whatever camera we get; that
measurement decides, not this table.

## 3. Field-version bill of materials

Specs marked **[datasheet]** were read from the manufacturer's page or datasheet
by web search on 2026-11-14. Everything else is **[unverified]**. Prices were
not researched except where a source is given; treat every price as
[unverified] and approximate.

| Part | Role | Spec that matters for 0.05 deg | What we found | Source |
| --- | --- | --- | --- | --- |
| Raspberry Pi High Quality Camera (Sony IMX477R), x2 | Mutual sighting | Pixel size and count | 12.3 MP, 1.55 um x 1.55 um pixels, 7.9 mm diagonal [datasheet]; 4056 x 3040 active pixels [datasheet, Arducam IMX477 page]. Price [unverified] | https://www.raspberrypi.com/products/raspberry-pi-high-quality-camera/ , https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/12MP-IMX477/ |
| 25 mm C-mount lens, x2 | Angular scale | Focal length, distortion | With the sensor above: 0.0036 deg/px, FOV 14.3 x 10.8 deg (computed). No specific lens checked [unverified] | computed |
| 850 nm IR LED, e.g. ams OSRAM SFH 4550, x2 | Beacon | Wavelength, beam angle | 850 nm infrared emitter [datasheet]; distributor lists it as "6 deg" (narrow beam) [unverified on the datasheet itself] | https://look.ams-osram.com/m/7d214b223a9adb85/original/SFH-4550.pdf |
| 850 nm band-pass filter, x2 | Daylight rejection | Pass band vs LED | Not researched [unverified] | - |
| Murata SCL3300-D01, x2 | Roll (and pitch) vs gravity | Offset stability, not resolution | Angle output 182 LSB/deg (0.0055 deg per LSB); offset error +/-0.57 deg over -40..+125 C (X, Y); offset lifetime drift +/-0.23 deg typical; noise density down to 15 ug/sqrt(Hz) depending on mode [datasheet] | https://www.murata.com/~/media/webrenewal/products/sensor/pdf/datasheet/datasheet_scl3300-d01.ashx?la=en-us |
| Analog Devices ADXL355 (alternative) | Same | Noise, offset drift | Noise density 22.5 ug/sqrt(Hz) on all axes [datasheet] = 0.0013 deg rms in 1 Hz (computed). Offset drift not read [unverified] | https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf |
| Benewake TF-Luna | Range | Range, accuracy | Operating range 0.2-8 m; accuracy +/-6 cm (0.2-3 m), +/-2% (3-8 m) [datasheet] | https://en.benewake.com/uploadfiles/2024/04/20240426135921367.pdf |
| Garmin LIDAR-Lite v3 / v3HP | Range | Range, accuracy | Range 40 m (70% reflective target), resolution +/-1 cm [datasheet]; v3HP accuracy +/-5 cm typical below 2 m [datasheet]; accuracy at 10-15 m not read [unverified] | https://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf , https://static.garmin.com/pumac/LIDAR-Lite_v3HP_Instructions_EN.pdf |
| Trinamic / ADI TMC2209, x4 | Stepper driver | Microstepping | Interpolation to 256 microsteps per full step [datasheet]; step input settings 8/16/32/64 by pins [unverified on the datasheet, from a vendor wiki] | https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2209_datasheet_rev1.09.pdf |
| NEMA17 stepper, 1.8 deg, x4 | Axis drive | Step angle | 1.8 deg assumed, no specific motor checked [unverified] | - |
| Worm-gear rotation stage, x2 | Pan, Tilt | Ratio, backlash | Example: 90:1 worm stage listed at $280; a precision stage (Newport RV120BPE) is listed at $10,191 with 0.001 deg feedback resolution. Listing excerpts only, backlash not confirmed [unverified] | https://optics-focus.com/product/motorized-rotation-stage/ , https://www.newport.com/p/RV120BPE |
| Lead-screw linear stage, +/-250 mm, x2 | X, Z | Travel, play | Not researched [unverified] | - |
| ESP32 dev board | Step pulses, limits, e-stop | - | Not researched [unverified] | - |
| Raspberry Pi 5 / laptop | Vision, control, UI | - | Not researched [unverified] | - |

### What the datasheets change in our design

1. **TF-Luna cannot be the field rangefinder.** Its range ends at 8 m; the field
   distance is 10-15 m. CHALLENGE.md suggests it; that row is wrong for the
   field. It is usable on a 2 m table.
2. **No rangefinder we found is good to a few mm,** and with matched offsets
   that no longer matters: a 60 mm range error costs 0.0017 deg at 2 m with a
   2 mm offset mismatch, against 0.083 deg with a 100 mm net offset. A tape
   measure is enough if the offsets are matched.
3. **The inclinometer is not a 0.005 deg absolute sensor.** 0.0055 deg is the
   SCL3300's output resolution; its offset error is up to +/-0.57 deg over
   temperature. CHALLENGE.md's "inclinometer ~0.005 deg" budget row confuses the
   two. It does not hurt us, because we use it for roll only: a 0.57 deg roll
   error costs 0.0006 deg at 2 m with a 2 mm offset mismatch (0.0285 deg with a
   100 mm net offset, one more reason for matched offsets). It does mean the
   inclinometer cannot serve as an independent elevation reference without its
   own calibration.
4. **A narrow-beam LED limits acquisition.** If the SFH 4550 beam is really a few
   degrees wide, a head that points more than that away from the other one shows
   a much dimmer beacon. The simulator assumes the beacon is visible anywhere in
   the camera's field of view. Either use a wider LED or check the brightness at
   the +/-1 deg setup error plus the spiral-search range.
5. **Worm stages span $280 to $10,000.** The cheap end has unverified backlash;
   closed-loop control on the camera tolerates backlash but not play that moves
   under wind.

## Still needed to finish this task

- The actual parts list and the demo distance. Then: redo sections 1 and 2 with
  real numbers, add dimensions measured on the built rig, and copy the real
  values into config.yaml with their source.
- A measured centroid sigma for the real camera (task 4).

## 4. Final design parts (2026-11-16)

Prices are the team's rough figures [team, unverified] unless a source is
given. Specs from docs/FINAL_DESIGN.md.

| # | Part | Qty | Role | Spec that matters | 2 m demo | Field 10-15 m |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Servo with optical encoder + driver | 6 | horn and radar Pan/Tilt/Roll | >= 0.001 deg per count at the output, low backlash, brake on the radar axes | same | same |
| 2 | Radar mount Pan/Tilt/Roll | 1 | carries 35 kg | CG on the axes; 10.3-34.3 N m for CG 3-10 cm off axis (arith) | radar mock-up | 35 kg |
| 3 | Horn mount Pan/Tilt/Roll | 1 | mechanics | pivot near the aperture centre; travel >= +/-5 deg | same | same |
| 4 | Dual-axis inclinometer | 2 | gravity, roll, R3 | accuracy, not resolution (section 3) | same | same |
| 5 | 2D PSD + front end | 3 | A, B (lens), C | S5991-01: 9 x 9 mm, 960 nm peak, 0.6 A/W, 1.5 um, +/-150 um, Isat 500 uA [datasheet] | S5991-01 is enough | A and C >= 20 mm (OSI DL-20 [vendor page]) or diffuser |
| 6 | Laser on the common board | 3 | light source | Class 1 at 960 nm, focusable 10 mm beam, attenuated to ~0.1 mW; invar/CFRP board | same | same |
| 7 | Raster scan | 0 | capture | horn servos, no separate scanner (Liam, 2026-11-16) | - | - |
| 8 | Camera with optical zoom | 1 | acquisition, KLT, UI | sees 960 nm or has a visible aid | webcam may do | zoom camera |
| 9 | Laser rangefinder | 1 | d | 10-15 m range | tape | rangefinder |
| 10 | ADC | 1 | PSD readout | 12 channels, 16-bit, >= 40 kS/s each | same | same |
| 11 | Controller | 1 | state machine, PID, UI | - | laptop | field PC |
| 12 | Calibration kit | 1 | CALIB | horn axis reference, targets | same | same |
| 13 | Cables, power, enclosures | - | infrastructure | cabled only | - | field grade |

Sums of the team ranges [arith]: USD 8,860-44,100; the high end exceeds the
USD 30k budget. Task 3 item 4 produces a <= 30k configuration from datasheets.
