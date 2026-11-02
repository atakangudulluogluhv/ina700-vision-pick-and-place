# Robot Setup

How to load `robot/MainModule.mod` and calibrate the station. Follow the steps in order the first
time. At a later start only the checks in Step 1 and Step 2 are needed.

## Names on the robot

| Name | What it is | Defined with | Defined from |
|---|---|---|---|
| `tool1` | Vacuum gripper | TCP calibration | TCP `[93.66, -36.08, 202.46]` mm, 1.1 kg |
| `wobj1` | Place fixture (the black mould) | **`tool0`** | Three corners of the mould |
| `wobj2` | Picking paper | **`tool1`** | Three corners of the paper, top-left = origin |

Signals: vacuum `doValve2`, yellow button `diOkButton`, lamp `doYellowLamp`.
Place points in `wobj1` (approach / grip): circle `p20/p30`, star `p40/p50`, hexagon `p60/p70`.

## Values from the last calibration (2026-11-02)

| Value | Where | Current |
|---|---|---|
| `wobj1` | robot | `[[712.991, 154.924, 599.812], [0.999976, 0.00178742, -0.00671454, 2.01148E-05]]` |
| `wobj2` | robot | `[[710.749, -190.346, 380.321], [0.999945, -0.00100401, -0.0104361, 0.000635222]]` |
| `coordinate.marker_offset_x_mm / y_mm` | Pi | 184.16 / 163.27 |
| `coordinate.pick_z_mm` | Pi | 6.8 |
| `coordinate.invert_x / invert_y / swap_xy` | Pi | true / false / false |
| `orientation.place_offset_deg` | Pi | star 23.0, hexagon 1.7, circle 0 |
| `TOOL_RZ_SIGN`, `PICK_Z_MIN_MM` | robot | -1, -10 |

## Editing RAPID from the PC

The quickest way is RobotStudio with the PC on `192.168.125.x`: Controller → Add Controller → online →
Request Write Access, open the module in the RAPID editor, edit the text and press Ctrl+S. This also
works for `wobjdata` values whose `trans` field is greyed out on the pendant.

## Step 1: Load the module

1. If a module you did not write is loaded, save it first: Program Editor → Modules → File →
   Save Module As.
2. Load `robot/MainModule.mod` (RobotStudio, or USB → File → Load Module).
3. Check that `ServerCommuncatio.sys` is loaded and that its `SocketConnect` address is
   `192.168.125.2`. Change nothing else in that file.
4. If `tool1` clashes with an existing `tool1`, delete the `tool1` line from `MainModule.mod`.
5. Compare `wobj1`, `wobj2` and `pPickTemplate` on the controller with the file. The controller once
   went back to an older module, which showed up as error 50050 "outside reach", so check this at
   every start.

## Step 2: `wobj2` and the marker offset

`wobj2` stays fixed to the robot; the camera measures relative to the marker. The Pi adds the marker's
position in `wobj2` to every pick.

1. Only if the paper was turned or `wobj2` is lost: Jogging → tool `tool1`, wobj `wobj2` → Define →
   3 points: top-left corner (origin), top-right (X), bottom-left (Y).
2. Jog `tool1` onto the marker centre with `wobj2` selected and read X/Y.
3. If they differ from the current offset by more than about 1 mm, enter them in the HMI's
   **CALIBRATION** tab, press APPLY, then SAVE TO CONFIG.

Axis directions are covered in [OFFSET_CALIBRATION.md](OFFSET_CALIBRATION.md).

If picks are still off-centre, look at the kind of error first:

| Error | Cause | Fix |
|---|---|---|
| Same direction and size everywhere on the paper | Paper or marker moved | Items 2 and 3 above |
| Grows with distance from the marker | Marker turned against `wobj2`, or not 100 mm wide | Measure the marker; redefine `wobj2` |
| Changes when the wrist turns | `tool1` TCP | Recalibrate the TCP |

## Step 2B: `wobj1` on the mould

Only needed if the mould was moved or turned. The place points are stored relative to `wobj1` and
follow it, as do the cavity angles on the Pi. The camera never sees the mould.

1. Select **`tool0`**. The place points were taught in a frame defined with `tool0`. Defining `wobj1`
   with `tool1` drops the frame by about 209 mm and shifts it sideways, which drives the tool into the
   mould. This has happened.
2. Jogging → wobj `wobj1` → Define → 3 points: touch the same three corners with the suction cup tip,
   using Linear jog only.
3. Check that Z is close to 600.

Teaching the place points again is only needed if the mould itself changed. Use tool `tool1`, wobj
`wobj1`, Linear jog only. Keep the wrist orientation: `pPickTemplate` must have the same orientation
as the place points, or every plate lands rotated by the difference.

## Step 3: Dry run

1. Set `coordinate.pick_z_mm: 60.0` for now. `wobj2`'s Z = 0 is the paper surface, so the robot then
   stops about 50 mm above the plates.
2. On the Pi: `python -m src.app.station --camera basler --fullscreen --no-verify`. The plate stays in
   place in a dry run, which would otherwise trigger a retry.
3. Pendant speed 25 %, start the program. The ROBOT lamp goes from WAITING to CONNECTED.
4. Choose a shape and colour, press PICK SELECTED, then the yellow button when its lamp lights up.
5. Is the suction cup exactly above the plate? If not, go back to Step 2.
6. Put `pick_z_mm` back.

## Step 4: Rotation direction (`TOOL_RZ_SIGN`)

Put a star on the paper rotated by about 30°. When it arrives above the cavity, are its tips aligned,
or rotated the wrong way by twice the angle? If wrong, set `TOOL_RZ_SIGN` to `1`.

## Step 5: Cavity angles (`orientation.place_offset_deg`)

1. Take a plate out of its cavity without turning it and lay it on the paper facing the same way.
   Keep only this plate and the marker on the paper.
2. In the HMI's CALIBRATION tab, under *Place angle*, select the shape, press USE LIVE and
   ADD TO OFFSET, then SAVE TO CONFIG.
3. The same plate should now read about 0.

The circle needs no angle.

## Step 6: Pick height (`coordinate.pick_z_mm`)

1. Jogging → tool `tool1`, wobj `wobj2`.
2. Lower the suction cup slowly until it touches the top of a plate lying on the paper.
3. Write the Z value into `pick_z_mm`. The module refuses anything below `PICK_Z_MIN_MM` (-10).

## Step 7: Full run

Run at 25 %. Once all three shapes land in their cavities, go up to 50-100 %. The console should show
`[recv] 'ACK;<id>;OK'` for every plate.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Robot hits the mould | `wobj1` was redefined with `tool1` | Step 2B; read `wobj1` before anything else |
| `ACK;..;FAIL` without the robot moving | Message hit a safety limit | Pendant line: `Outside pick area: x/y` → offset or axis error; `: z` → `pick_z_mm`; `No place cavity` → shape name |
| `Rejected: ...` | Malformed message | Is `comm.message_template` in the PICK format? |
| `no answer from the robot within 120 s` | Yellow button not pressed | Press it, or set `USE_OK_BUTTON := FALSE` |
| Configuration error during a pick | Wrist limit | Reduce `orientation.wrist_limit_deg` |
| Plate drops | Lifts before the vacuum has built up | Increase `VACUUM_SETTLE_SEC` |
| Suction cup off-centre by the same amount everywhere | Marker offset | Step 2 |
| Always the same wrong angle in the cavity | Cavity angle | Step 5 |
| Rotated the wrong way, worse as the angle grows | Sign | Step 4 |
| Robot goes to the old place after the mould moved | `wobj1` out of date | Step 2B |
