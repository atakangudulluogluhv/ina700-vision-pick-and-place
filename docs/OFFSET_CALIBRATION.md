# Offset Calibration — Step by Step

> Working document (not part of the submission package).
> Goal: the `x_mm, y_mm` reported by the camera and the X, Y the robot sees inside its pick work object
> should be **the same numbers**.
>
> **Names on the robot** (taught 2026-09-28, see `ROBOT_SETUP.md`): tool = **`tool1`** (vacuum),
> pick wobj = **`wobj2`** (the paper), place wobj = **`wobj1`** (the mould), vacuum = **`doValve2`**,
> yellow button = **`diOkButton`**.
>
> **Done on 2026-09-28:** axis directions `invert_x: true`, `invert_y: false`, `swap_xy: false`;
> `wobj2.oframe.trans` moved onto the marker (5B) and verified ~0,0 at the marker. Live picks were then
> still a few mm off-centre by the same amount everywhere, corrected twice to **`814.441, 3.83`**
> (in `robot/MainModule.mod`, not yet loaded on the robot). Next: load it and verify with a pick.

---

## What is the offset? (30 seconds)

The camera measures the object's position **relative to the centre of the marker**.
The robot interprets numbers **relative to the origin of `wobj2`** (the first point touched when the wobj was taught).

If those two points are not the same place, the robot goes to the wrong spot **by the same amount every time**.
That constant difference is called the **offset**.

```
   wobj2 origin (robot's zero)
   ●───────────────── X
   │        ▓▓▓
   │        ▓●▓  ← marker centre (camera's zero)
   │        ▓▓▓
   │    distance between them = OFFSET
   Y
```

---

## ⚠️ Know this first

`settings.yaml` has **no `offset_x_mm` / `offset_y_mm` fields** (as of 2026-09-17).
If step 5 finds a non-zero offset there are two ways:

- **If the code has been added:** you will see these two fields under `coordinate:` in `settings.yaml` → enter the values.
- **If not:** the **pendant method** in step 5B fixes it on the robot side without touching code.

If you write a field under `coordinate:` that the code does not read, **nothing happens and no error is raised.**
Do not write it and assume it is fixed; that is why step 6 verifies.

---

## Step 0 — Preparation

1. **Tape** the paper and the marker to the table. If the paper slides, everything starts over.
2. Measure the black outer edge of the marker with a ruler. **If it is not 100 mm**, correct
   `aruco.marker_length_mm` in `settings.yaml`. (If it is wrong, every mm value is wrong by the same ratio.)
3. Put **only one object** on the paper. With several objects you will lose track of which one the program chose.
4. Open a terminal on the Pi:
   ```bash
   cd ~/INA700_project
   source ~/INA700_venv/bin/activate
   python -m src.app.main --camera basler --no-robot --template "PICK;{id};{shape};{color};{x_mm};{y_mm};{z_mm};{rz_deg}"
   ```
5. In the window you should see a **red line (X)** and a **blue line (Y)** on the marker.
   - If not, the console prints `[aruco] marker not found` → check the light, glare, and whether something covers the marker.
   - To quit, press `q` with the window focused.

---

## Step 1 — Ask the camera

1. Put the object on the **left side** of the paper.
2. Every few seconds the console prints a line like this:
   ```
   [send] (no robot) 'PICK;1;square;green;-150.2;210.5;0.0;12.0'
   ```
3. **Fields 5 and 6** (`-150.2` and `210.5`) are the camera's answer. Write them in the table.

---

## Step 2 — Ask the robot

1. Teach pendant → **Jogging** screen.
2. Select **Coordinate system: Work object**.
3. Select **Work object: `wobj2`**.
   ⚠️ **Most common mistake:** if `wobj0` stays selected the numbers are meaningless.
4. Select **Tool: `tool1`** (not `tool0`). `tool1`'s TCP is the tip of the suction cup.
5. Jog the robot **slowly** until the suction cup is **exactly over the centre** of the object.
   Do not touch it; stop 1–2 mm above. (A pointer tool also works if it is more precise,
   but then select **that tool's** name.)
6. Read X and Y from the **Position** field on the right of the pendant and write them in the table.
7. Raise the robot and move it away from the object.

---

## Step 3 — Collect 3 points

Put the object at **left → right → bottom** positions and repeat step 1 + step 2 at each.
The points should be **far apart** (at least 150–200 mm).

| Point | Camera X | Camera Y | Robot X | Robot Y |
|---|---|---|---|---|
| left | | | | |
| right | | | | |
| bottom | | | | |

*Example filled table:*

| Point | Camera X | Camera Y | Robot X | Robot Y |
|---|---|---|---|---|
| left | −150 | 210 | 50 | 180 |
| right | 140 | 205 | 340 | 175 |
| bottom | 0 | 400 | 200 | 370 |

---

## Step 4 — Are the directions right? (do this BEFORE the offset)

Look at the **left → right** rows:

| What you see | What to do (`settings.yaml` → `coordinate:`) |
|---|---|
| Camera X grew, robot X grew too | ✅ leave it |
| Camera X grew, robot X **shrank** | `invert_x: true` |
| Camera **X** changed but robot **Y** changed | `swap_xy: true` |

Look at the **left → bottom** rows (same for Y):

| What you see | What to do |
|---|---|
| Camera Y grew, robot Y grew too | ✅ leave it |
| Camera Y grew, robot Y **shrank** | `invert_y: true` |

If you changed a setting → **close the program (`q`), start it again, and redo steps 1–3.**
Computing an offset before the directions are right is meaningless.

> 💡 Check: an **odd number** (1 or 3) of `invert_x`, `invert_y`, `swap_xy` is expected to be `true`
> (the camera looks down, the robot's Z points up). If you get an even number, check the measurement again.

---

## Step 5 — Compute the offset

For each row, **Robot − Camera**:

| Point | Robot X − Camera X | Robot Y − Camera Y |
|---|---|---|
| left | 50 − (−150) = **200** | 180 − 210 = **−30** |
| right | 340 − 140 = **200** | 175 − 205 = **−30** |
| bottom | 200 − 0 = **200** | 370 − 400 = **−30** |

Depending on the result:

| Case | Meaning | What to do |
|---|---|---|
| All three rows **close to 0** (±5 mm) | `wobj2` is already at the marker centre | 🎉 No offset, go to step 6 |
| All three rows **the same number** (±5 mm), not 0 | There is an offset | 5A or 5B below |
| Rows **very different** from each other | Not an offset, a different problem | Back to step 4; check that `wobj2` and `tool1` are selected |
| Difference **grows** as points get farther apart | Scale error | Step 0.2: is the marker size right? |

Take the average: in the example `offset_x = 200`, `offset_y = −30`.

### 5A — If the code has been added (preferred)

`config/settings.yaml`:
```yaml
coordinate:
  offset_x_mm: 200.0
  offset_y_mm: -30.0
```

### 5B — If not: shift `wobj2` itself

Same result without touching code. Idea: move the robot's origin to the marker centre.
`wobj2` belongs to this project only, so it can be shifted freely. **Never shift `wobj1`** (the mould).

1. Select **`wobj2`** → **Edit** → **Change Value** → **`oframe`** → **`trans`**.
   If `trans` is greyed out on the pendant, edit the `wobj2` line in **RobotStudio online** instead
   (`ROBOT_SETUP.md`, "Editing RAPID from the PC").
2. Check that `oframe.rot` is **`[1, 0, 0, 0]`**. If not, **stop**: this method does not apply.
3. **Add** the offset to the current values:
   - `oframe.trans.x` = old value **+ 200**
   - `oframe.trans.y` = old value **+ (−30)**
4. Save and **take a photo**. Copy the new value into `robot/MainModule.mod` so the repo matches the robot.
5. Repeat steps 1–3 → the difference should be **close to 0**.

*2026-09-28, for reference:* the tool was jogged onto the marker with `wobj2` selected, the Position
read (118.56, 145.82) and that was added onto the previous `oframe.trans` (714.881, −144.99).

---

## Step 6 — Verify (DO NOT SKIP)

1. Put the object somewhere **you have not used before**, preferably in a **corner** of the paper.
2. Restart the program and read the X, Y given by the camera.
3. Jog on the pendant (Work object: `wobj2`, tool: `tool1`) and bring the suction cup to **those numbers**.
4. Is the suction cup over the centre of the object?
   - **Within ±5 mm** → ✅ done. Save `settings.yaml` and **take a photo**.
   - **No** → back to step 3.
   - **Off in the corner but right in the middle** → lens distortion (risk B10). Note the value; it will be solved at home.
     Do not spend lab time on it.

---

## Reminder card

```
0. Paper taped? Marker 100 mm? Single object?
1. Camera X,Y  → fields 5 and 6 of the PICK line in the console
2. Robot X,Y   → pendant: Work object = wobj2, tool = tool1
3. 3 points: left, right, bottom
4. Direction reversed?  → invert_x / invert_y / swap_xy   (then measure again)
5. Robot − Camera always the same? → offset (5A config / 5B pendant)
6. Test at a new point (a corner)
```

## Before leaving the lab

- [ ] Final `settings.yaml` saved + photographed
- [ ] Table of the 3 points (photo)
- [ ] If 5B was done: `wobj2` values written down and copied into `robot/MainModule.mod`
- [ ] `pick_z_mm`: Z in `wobj2` when the suction cup sits on an object **on the paper**
      (−195.83 on 2026-09-28)
- [ ] Step 6 verification done
