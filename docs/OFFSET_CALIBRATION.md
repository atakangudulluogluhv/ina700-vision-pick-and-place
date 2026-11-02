# Calibrating Camera Positions Against the Robot

Goal: the `x_mm, y_mm` the camera reports and the X, Y the pendant shows in `wobj2` should be the same
numbers. Two things have to be right, the axis directions and the marker offset.

The camera measures a plate relative to the centre of the ArUco marker. The robot works relative to the
origin of `wobj2`, the paper's top-left corner. The difference between the two points is
`coordinate.marker_offset_x_mm / y_mm` in `config/settings.yaml`.

```
wobj2 origin
+------------------- X
|         ###
|         #o#  <- marker centre, at (marker_offset_x_mm, marker_offset_y_mm) in wobj2
|         ###
Y
```

## Preparation

1. Tape the paper and the marker to the table.
2. Measure the marker's black outer edge with a ruler. If it is not 100 mm, correct
   `aruco.marker_length_mm`, otherwise every distance is off by the same ratio.
3. On the Pi:

   ```bash
   python -m src.app.main --camera basler --no-robot
   ```

   The window shows a red line (X) and a blue line (Y) on the marker. Press `q` to quit.
   The console prints a line like `[send] (no robot) 'PICK;1;star;green;150.2;210.5;6.8;12.0'`;
   fields 5 and 6 are the camera's X and Y.

## Step 1: Axis directions

Only needed if the camera was moved or turned. The current values are `invert_x: true`,
`invert_y: false`, `swap_xy: false`.

Put one plate on the left of the paper, then on the right, and then near the bottom. At each position
write down the camera's X, Y, and the pendant's X, Y with the suction cup above the plate's centre
(pendant: coordinate system Work object, wobj `wobj2`, tool `tool1`).

| What you see | Setting |
|---|---|
| Camera X grew and robot X shrank | `invert_x: true` |
| Camera Y grew and robot Y shrank | `invert_y: true` |
| Camera X changed but robot Y changed | `swap_xy: true` |

Restart the program after changing a setting and measure again. The camera looks down and the robot's
Z points up, so an odd number of these three settings should be true.

## Step 2: Marker offset

1. Jog `tool1` onto the marker centre with `wobj2` selected and read X and Y on the pendant.
2. Enter them in the HMI's CALIBRATION tab (or directly in `settings.yaml`).
3. APPLY takes effect from the next frame without a restart. SAVE TO CONFIG keeps the value for the next
   start.

Do this whenever the paper may have moved.

## Step 3: Verify

Put a plate near a corner of the paper, somewhere you have not used yet. Read its X, Y from the camera,
then jog the suction cup to those numbers. It should be above the plate's centre.

| Result | Meaning |
|---|---|
| Same error everywhere | The offset is wrong; repeat Step 2 |
| Error grows with distance from the marker | Scale or rotation; check the marker's size and that it lies parallel to the paper edge |
| Numbers do not match at all | Wrong wobj or tool selected, or an axis setting is wrong |
