# Robot Setup — Final Module (`robot/MainModule.mod`)

> Follow these steps **in order** in the lab. Each step relies on the previous one; do not skip any.

## Work objects and names on the robot

Taught on the pendant on 2026-09-28. **This is the reverse of the original plan** (which called the
paper `wobj_pick` and the mould `wobj2`); the module and every document now use these names:

| Name | What it is | Defined from |
|---|---|---|
| **`wobj1`** | Place fixture (the black mould) | The mould's own three corners (Step 2B) |
| **`wobj2`** | Picking paper | Origin moved onto the ArUco marker centre (Step 2) |
| **`tool1`** | Vacuum gripper | TCP `[93.66, -36.08, 202.46]` mm, 1.1 kg |

Signals: vacuum **`doValve2`**, yellow button **`diOkButton`**, lamp **`doYellowLamp`**.
Place points: circle `p20/p30`, star `p40/p50`, hexagon `p60/p70` (approach / grip), all in `wobj1`.
`robot/robot_code_final.mod` is the module exported from the robot at the end of that lab.

## Status (2026-10-07)

| Step | | Result |
|---|---|---|
| 0 — Back up old module | ⏭️ skipped | The robot had no module loaded |
| 1 — Load the module | 🟡 | Loaded (= `robot/robot_code_final.mod`), but with the older `wobj2` (see Step 2) |
| 2 — `wobj2` on the paper | ✅ | 2026-10-07: taught on the paper's corners (top-left origin), `[[626.676, -153.111, 385],[1,0,0,0]]`. Marker centre in this frame (206.79, 153.91) → Pi `coordinate.marker_offset_x/y_mm`. Picks land centred |
| 2B — `wobj1` on the mould + place points | ✅ | Mould corners taught, `p20…p70` re-taught |
| 3 — Dry run | — | Not recorded; live picks were run directly |
| 4 — `TOOL_RZ_SIGN` | 🟡 | Suspect: hexagon at rz 24° did not seat (wrong sign gives 2×rz). Test with a hexagon turned ~15° |
| 5 — Cavity angles | ✅ | Re-measured 2026-10-07: star **26.9**, hexagon **1.4**, circle 0. `pPickTemplate` now equals the place orientation (was 13.6° off) |
| 6 — Real Z | ✅ | `pick_z_mm: 6.8` in the paper-corner `wobj2`; `PICK_Z_MIN_MM := -10` |
| 7 — Full run | 🟡 | Picks centred; star placed well, hexagon sometimes off → Step 4. Note `PICK_REACH_MM` 400 vs paper x up to ~405 |

Also measured: `coordinate.invert_x: true`, `invert_y: false`, `swap_xy: false` (the camera is mounted turned).

---

## Editing RAPID from the PC (preferred)

**RobotStudio → Controller → Add Controller → online** (the PC on `192.168.125.x`) → **Request Write
Access** → open the module in the RAPID editor, edit the text, **Ctrl+S** writes it to the controller.
Verified on 2026-09-28. This is faster than the pendant menus, and it also works for `wobjdata`
values whose `trans` field is greyed out on the pendant.

## Step 0 — Do not lose the old module

Only if a module you did not write is loaded: Pendant → **Program Editor** → **Modules** →
**File → Save Module As...** → USB stick → add it to `robot/archive/` at home.

## Step 1 — Load the module

1. Load `robot/MainModule.mod` (RobotStudio online editor, or USB → **File → Load Module...**).
2. **Do not touch** `ServerCommuncatio.sys` (T18). Check that it is loaded and that its `SocketConnect`
   IP is `192.168.125.2`.
3. If loading fails: note the line number and check which signal/name could not be found.
   If `tool1` clashes with an existing `tool1`, delete the `tool1` line from `MainModule.mod`.
4. Check that the loaded `wobj1`, `wobj2` and `p20…p70` match the repo file (they are `TASK PERS` /
   `CONST`; a reload restores the file's values).

## Step 2 — Put `wobj2` on the marker

Follow **`docs/OFFSET_CALIBRATION.md`**: axis directions (step 4), then move `wobj2.oframe.trans`
onto the marker centre (step 5B), then verify (step 6).

If live picks are still off-centre afterwards, check the **kind** of error before changing anything:

| Error | Cause | Fix |
|---|---|---|
| Same direction and size everywhere on the paper | `wobj2` origin not exactly on the marker | Add it to `wobj2.oframe.trans` (see below) |
| Grows with distance from the marker | Marker turned against `wobj2`'s X axis, or marker not 100 mm | Measure the marker; redefine `wobj2` along the marker |
| Changes when the wrist turns | `tool1` TCP | Recalibrate the TCP |

To add a constant correction: measure it on the plate along the **marker's** axes (red = X, blue = Y
on screen). Because `invert_x: true`, add **−X** and **+Y** to `wobj2.oframe.trans`.

## Step 2B — Define the mould as `wobj1` (done 2026-09-28)

1. **Mark 3 corners** on the mould: **O** = origin, **X** = the other corner on the same edge,
   **Y** = O's neighbour on the other edge. **Photograph** which corner is which.
2. Pendant → **Jogging** → Work object → **`wobj1`** → **Edit → Define...** → User method: **3 points**
   → touch **O, X, Y** with the TCP → **Modify Position** → OK.
3. **Re-teach the place points in `wobj1`:** `p20`…`p70` → suction cup into each cavity
   (tool `tool1`, wobj **`wobj1`**) → **Modify Position**. Approach points are ~50 mm above.
   ⚠️ **Do not rotate the wrist while teaching** — use **Linear** jog only. The release orientation must
   stay equal to the pick template `pPickTemplate`, otherwise the difference is added to every placement.
   (On 2026-09-28 the re-taught points differ from `pPickTemplate` by ~13.6° in yaw, yet the plates
   seated correctly; confirm in Step 4.)
4. Save. `TASK PERS` → `wobj1` persists.

**If the teacher moves / rotates the mould:** repeat **item 2 only** with the same O-X-Y corners.
The points and the cavity angles on the Pi follow `wobj1` untouched. The camera never sees the mould.

## Step 3 — Dry run (in the air)

Set `config/settings.yaml` → `coordinate.pick_z_mm: 0.0` **temporarily**. The robot then "picks"
~200 mm above the paper and cannot hit anything. Put it back to **`-195.83`** afterwards.

1. On the Pi: `python -m src.app.station --camera basler --fullscreen --no-verify` (server **first**;
   the ROBOT lamp goes **WAITING** → **CONNECTED**). Choose shape + colour → **PICK SELECTED**.
   `--no-verify`: the plate stays where it is in a dry run, which would otherwise trigger a retry.
2. Pendant speed **25 %**, start the program. When the yellow lamp lights up → press the **yellow button**.
3. Check: is the suction cup **exactly above** the chosen plate? If not → Step 2.

## Step 4 — Is the rotation direction right? (`TOOL_RZ_SIGN`)

1. Put a **star** on the paper, rotated by ~30°.
2. When the star is lifted and brought above the cavity, are its tips **aligned** with the cavity,
   or rotated the wrong way by twice the angle?
3. If it turned the wrong way: `MainModule.mod` → `CONST num TOOL_RZ_SIGN := -1;` → **`1`**, save.

## Step 5 — Cavity angles (`orientation.place_offset_deg`) (done 2026-09-28)

1. Take a plate out of its cavity **without turning it** and lay it on the paper facing the same way.
   Keep **only this plate + the marker** on the paper.
2. Read its angle on the HMI (or `python -m src.app.main --camera basler --no-robot` → **space**,
   last number of the `[send]` line).
3. `settings.yaml` → `orientation.place_offset_deg.<shape>` = that number. Restart the program
   (settings are read only at start-up).
4. Verify: the same plate should now read **~0**.

Not needed for the circle.

## Step 6 — Real Z (`coordinate.pick_z_mm`) (done 2026-09-28: −195.83)

1. Pendant: **Jogging** → tool `tool1`, work object **`wobj2`**.
2. Lower the suction cup slowly until it **touches the top surface** of a plate on the paper.
3. Read **Z** → write it into `pick_z_mm`.
4. The module **rejects anything below −215** (`PICK_Z_MIN_MM`).

## Step 7 — Full run

Speed 25 % → once all three shapes land in their cavities, 50–100 %. Every cycle the console should show
`[recv] 'ACK;<id>;OK'`.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ACK;..;FAIL` without the robot moving | Message hit a safety limit | Pendant line: `Outside pick area: x/y` → offset/axis error; `: z` → `pick_z_mm`; `No place cavity` → shape name |
| `Rejected: ...` | Malformed message | Is `comm.message_template` in the PICK format? |
| `no answer from the robot within 120 s` on the Pi | Yellow button not pressed | Press it, or set `USE_OK_BUTTON := FALSE` |
| Configuration error during pick | Wrist limit | Reduce `orientation.wrist_limit_deg` |
| Plate drops | Lifts before vacuum builds up | Increase `VACUUM_SETTLE_SEC` (0.5 → 1.0) |
| Suction cup lands off-centre, same amount everywhere | `wobj2` origin | Step 2 |
| Always lands in the cavity at the same wrong angle | Offset | Step 5 |
| Rotated the wrong way, worse as the angle grows | Sign | Step 4 |
| Mould moved, robot goes to the old place | `wobj1` is stale | Step 2B, item 2 (same 3 corners) |
| `wobj2` `trans` greyed out on the pendant | Pendant restriction | Edit it in RobotStudio online |
