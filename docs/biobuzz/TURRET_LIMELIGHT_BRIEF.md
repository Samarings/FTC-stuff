# Turret + Limelight 3A brief (FTC BIOBUZZ 2026-27)

> **Superseded hardware:** the robot is now the **underhand design**; see
> `UNDERHAND_BRIEF.md`. That file overrides this one for CAD/hardware, camera pose,
> turret limits (now ±225° with a 360° flip), turret controls and aiming. Everything
> else here (field coordinates, localization, shot solver, TIP tracking, AUTO, rules,
> DECODE reuse) still applies.

Context for Claude Code: we are writing Limelight 3A code for our turret shooter.
Everything under "Hardware (from CAD)" was measured from our Onshape assembly
("Turret maybe" → Assembly 1). Items marked **TODO** are unknown and must be asked
of the team or measured before hard-coding. Full game rules are in
[BIOBUZZ_RULES.md](BIOBUZZ_RULES.md) in this same folder (`docs/biobuzz/`).

## Goal

1. Auto-aim the turret (yaw) at our alliance's **upward-facing HIVE CELL** using the
   Limelight 3A.
2. Estimate distance to that CELL and set flywheel velocity from a tuned lookup table
   (the hood is fixed, so velocity is the only distance knob).
3. **Localization:** Pinpoint odometry is the main pose and heading source, and the
   Limelight corrects x/y every few seconds. The camera is mounted on the turret, not
   the chassis, which the correction must account for. See "Localization (odometry +
   Limelight)".
4. Only allow a shot when the target is valid and aim error is small (see "Rules that
   constrain the code").
5. **Shoot from anywhere.** One shot solver (odometry + Limelight) that gives turret
   angle and flywheel RPM from any field position. Teleop and AUTO share it.
6. **Autonomous.** A 30 s AUTO that leaves the wall, shoots the 4 pre-loads, tips the
   HIVE, collects and shoots more POLLEN, and parks. See "Autonomous (AUTO)" below.

## ⚠ TEMPORARY: treat the turret as FIXED (does not rotate)

**For now the turret does not rotate.** Write the code as if the shooter and Limelight
are bolted to the chassis facing robot-forward, and **aim by turning the whole robot**,
like DECODE did. This is temporary: the rotating turret comes back later. So build it
behind one switch, and keep the rotating-turret design in this brief intact.

**Switch:** `RobotConstants.TURRET_FIXED = true`. Switching to the rotating turret later
should only need this flag set to `false` plus the turret calibration values. No
rewrites.

**While `TURRET_FIXED = true`:**
- **Turret angle** is the constant 0° (home = camera facing robot-forward). The turret
  motor is **not required** in the hardware map: look it up optionally, and never
  power it if found. `Turret.update()`, rotation limits, turret calibration and
  cable-wrap logic are inactive.
- **The team should physically lock the turret at its home mark** (pin, bolt or zip
  tie) so it can't drift.
- **Camera pose** is fixed relative to the chassis. Set the Limelight "camera pose in
  robot space" relative to the **robot center on the floor**: forward =
  `TURRET_AXIS_OFFSET.forward` + 0.1165 m, side = `TURRET_AXIS_OFFSET.left`, up =
  `CAMERA_HEIGHT_M` (≈ 0.354 m, i.e. lens height above the tiles, not the 0.0688 m
  above the bearing), pitch +37°. MegaTag2 then returns the **robot center**
  directly, so do **not** subtract `TURRET_AXIS_OFFSET` again in code (see
  "Converting the result to a robot pose"). The MegaTag2 yaw input is just
  `robotHeadingDeg`, converted to the Limelight field frame.
- **Shot solver** still computes the same `desiredYaw` (direction to our up CELL
  relative to the robot) and RPM. But instead of `Turret.chooseSetpoint()`, it outputs
  a **desired robot heading** = `robotHeading + desiredYaw`. There's no dead zone: the
  chassis can always turn to face the target.
- **Teleop aiming:** reuse DECODE's chassis-aim pattern from `MainTeleop.java`.
  - Driver toggle: `computeLimelightAimRotate()` for tx when tags are visible,
    `computeAimToPointRotate()` from odometry otherwise.
  - The aim controller **only takes over rotation**. The driver keeps field-centric
    translation, so they can strafe while staying aimed.
  - Replace the target logic with the BIOBUZZ cluster/up-CELL logic, and the odometry
    goal with the up CELL's field position.
  - Retune gains (new robot).
- **AUTO aiming:** at each shooting spot, Pedro paths end with the heading that faces
  the target. Compute it from the field positions with the solver, not hand-typed
  angles. Then hold or fine-correct heading with the Limelight before firing.
  Shooting while moving only works when the path heading already points at the
  target, so it's a stretch goal in this mode.
- **`canShoot`** uses **robot heading error** < tolerance (plus low turn rate) in
  place of turret error. `turretCanReach` is always true.
- **Localization** is the same plan, with turret angle 0 and no turret turn-rate gate.
  The Limelight correction gates still use robot turn rate.

**Code structure so switching is easy:** keep an `Aimer` interface with two
implementations:
- `ChassisAimer` (fixed turret, used now): turns the robot.
- `TurretAimer` (rotating turret, later): `Turret.chooseSetpoint()` + limits.

The shot solver, AUTO and teleop only call the interface. All rotating-turret sections
below (rotation limits, turret calibration, turret-frame camera pose) describe the
**later** setup. Implement them behind the flag or leave them for later, but don't
delete them.

## Hardware (from CAD)

CAD frame: Z is up, the ring bearing lies flat in the XY plane. Numbers are mm unless noted.

| Item | What the CAD shows |
|---|---|
| Turret bearing | Large ring bearing, outer race Ø144 mm, inner race Ø124 mm, ~10–13 mm thick. Center at CAD (X 84.1, Y 33.25). Open center, so balls appear to feed up through it. |
| Turret yaw axis | Vertical (CAD Z) through the bearing center. |
| Turret drive | **Not modeled.** No motor, gear or belt drives the bearing in CAD. The team doesn't know the motor, ratio or ticks/deg yet. Use the placeholders and `TurretCalibration` in "Unknown values". |
| Rotation range | **Limited in software to protect the wiring. See "Turret rotation limits (cable wrap protection)".** CAD gives no limits (all 27 mates are Fastened). Cables cross the rotating joint: the Limelight USB **and** the flywheel 5203's power + encoder cables (that motor is mounted on the turret). |
| Flywheel RPM | Tunable at any time. All RPM values (cap, tables, idle) are constants the team will edit after testing; don't treat any number as final. |
| Shooter | Hooded single-stage shooter: two side-by-side wheels (each Ø73.4 mm ≈ 2.89 in, 24 mm wide) on one 8 mm REX shaft (120 mm) at CAD (Y −20.5, Z 68.6), with a **fixed** curved hood behind them (CAD Y 34–87, Z 69–132). The ball appears to be squeezed between wheels and hood and leaves upward and forward (toward CAD −Y, same way the camera faces). **Verify exit direction with the team.** |
| Flywheel motor | One goBILDA 5203 Yellow Jacket on the back (shaft along X at CAD Y 111, Z 57.5). Not coaxial with the wheel shaft (≈132 mm apart), so there is a belt/chain/gear not modeled. The ratio is unknown but **not needed**: all RPM values are motor-encoder RPM. |
| Flywheel speed limits (from the team) | **Hard max 6000 RPM** (the motor's top speed). **Normal operating cap 3500 RPM.** The team expects never to need more than 3500 for POLLEN. Clamp every commanded RPM to `[0, 3500]` by default, and keep a single constant `RPM_CAP` that can be raised (never above 6000) if testing shows far shots fall short. RPM is measured at the motor encoder (DECODE used 28 ticks/rev). |
| Side plates | Two 6 mm plates at CAD X≈29 and X≈139 (pocketed). |
| Limelight 3A | Mounted on the turret's front plate, fastened to the turret, so **it rotates with the turret**. |
| Test ball in CAD | Ø76.2 mm (3.0 in) placeholder. Real POLLEN is ~2.8 in (71 mm), NECTAR ~3.6 in (91 mm). |

### Camera pose relative to the turret (measured)

Define a **turret frame**: origin on the yaw axis at the bottom face of the ring bearing,
+forward = the direction the camera faces (CAD −Y), +left = CAD +X, +up = CAD +Z.

| Value | Measurement |
|---|---|
| Forward offset from yaw axis | **116.5 mm (4.59 in, 0.1165 m)** |
| Left/right offset | **0** (lens is on the turret centerline) |
| Height above bearing bottom face | **68.8 mm (2.71 in, 0.0688 m)** |
| Pitch | **+37° up** (lens normal = (0, −0.799, +0.602) in CAD) |
| Yaw / roll relative to turret forward | 0° / 0° |

Measured to the center of the Limelight's front glass, so the optical center may be off
by a few mm. Verify pitch sign conventions in the Limelight web UI 3D view.

**Heights above the floor (estimated from the team's info):** the turret sits on top
of the robot, and robot + turret is **about 16 in (406 mm) tall**. In CAD the highest
turret part (top of the hood) is 121.2 mm above the bearing's bottom face. So:
- Bearing bottom face ≈ 16 in − 4.77 in = **≈ 11.2 in (285 mm)** above the tiles.
- **Limelight lens ≈ 285 + 68.8 mm = ≈ 354 mm (13.9 in)** above the tiles. Use
  `CAMERA_HEIGHT_M = 0.354`.
- Ball exit is near the top of the hood, **≈ 15–16 in (0.39–0.41 m)** above the tiles.
- These are estimates. Measure the real lens height on the built robot and update the
  constant.

**Turret-axis offset from robot center (`TURRET_AXIS_OFFSET`, forward/left in inches):**
the team hasn't measured it. **Default `(0, 0)` and keep it a named constant.** Where
it matters:
- **Aiming while our tags are visible:** it doesn't matter. The turret closes the loop
  on Limelight tx, which is measured from the turret itself.
- **Aiming from odometry alone** (tags out of view, too close, blocked): it matters.
  The yaw error is about `offset × sin(angle) / distance`. A 4 in offset at 60 in
  distance gives up to ~4°, and the CELL opening allows only ~5–6° at that range.
- **Limelight pose corrections:** it matters. The MegaTag2 pose is the turret's
  position, so a wrong offset puts the corrected robot position off by up to the
  offset size.

So it's optional at first, but it's a 2-minute tape-measure job (robot rotation center
→ bearing center) that makes "shoot from anywhere" and localization more accurate.
Put a reminder in telemetry while it's still `(0, 0)`.

**Turret encoder zero:** which turret angle is "camera faces robot forward". The team
picks the home mark (see "Turret rotation limits").

### Field of view check

Limelight 3A FOV is about ±27° horizontal (tx) and ±20.5° vertical (ty). With a 37° pitch,
the camera sees roughly 16.5° to 57.5° above horizontal. The tags are about 4 ft up and
face downward (see below), so a steep upward camera suits this game.

Rough numbers:
- **Cluster height:** ≈ 45 in (1.14 m). This is estimated from the HIVE figures (up
  CELL's floor); measure it from the field CAD.
- **Camera height:** ≈ 13.9 in.
- **Height difference:** ≈ 31 in (0.79 m).
- **Range where the cluster is in the vertical FOV:** about **20 in to 105 in**
  (0.5–2.7 m) horizontal distance.

Too close (nearly under the HIVE) the target leaves the top of the frame. Far away the
view of the downward-facing tags gets very oblique, so detection will drop out before
the geometric limit. Plan to coast on odometry plus the last known target when tags drop
out.

## BIOBUZZ vision facts that matter for this code

Source: BIOBUZZ Competition Manual TU07, §9.6 and §9.9 (field figures checked).

- **Targets.** There is 1 central HIVE Structure holding a red HIVE and a blue HIVE.
  Each HIVE is a rocker with 2 CELLS, one on each end. Exactly one CELL per HIVE faces
  up at a time, and you score by LAUNCHING into the **upward-facing CELL**. Enough
  balls in it **TIPS** the HIVE, which swaps which CELL is up.
- **AprilTags.** 36h11 family, 3.25 in (82.5 mm) squares, in **clusters of 4 on one
  sticker**. One cluster is on the **bottom face of each CELL, facing down toward the
  tiles**, with its bottom edge toward field center. **Tags move when the HIVE tips.**
- **IDs:**

  | IDs | CELL |
  |---|---|
  | 30, 31, 32, 33 | Red CELL, far side (opposite the audience) |
  | 34, 35, 36, 37 | Red CELL, audience side |
  | 38, 39, 40, 41 | Blue CELL, audience side |
  | 42, 43, 44, 45 | Blue CELL, far side |

- **Using the IDs.** Only an upward-facing CELL's bottom faces the floor, so in normal
  play **the cluster you can see marks the CELL you should shoot into.** If our
  alliance's visible cluster changes, a TIP happened. The other alliance's tags are
  also visible, so always filter by our alliance's IDs.
- **Cluster layout** (along the sticker): tag centers are **±2.75 in and ±6.5 in** from
  the cluster center. The cluster center is 2.75 in from the line through the two
  0.5 in reference holes. That reference-hole line is 9.938 in from the "front" edge of
  the CELL. **Aim at the cluster center**, i.e. the average of the visible tags corrected
  by their known offsets, not at a single tag.
- **HIVE geometry.**
  - CELLS tilt 30° in each stable state, and each HIVE's two CELLS are ~18.8 in apart.
    **⚠ This disagrees with the field table below** (CELLS at y ≈ 56.6 and 87.4, i.e.
    30.8 in apart; 18.8 in would put them at y ≈ 62.6 and 81.4). Neither is confirmed.
    Take the CELL opening centers from the official field CAD before tuning aim.
  - The pivot axis is 43.95 in above the tiles.
  - The upward-facing CELL's opening spans 53.5 in to 65.6 in above the tiles.
  - The opening is ~20 in wide × 14 in tall × 12 in deep.
  - The red and blue HIVES are 25.5 in apart center to center.
  - The ball must go **into the opening**, which sits above and outward of the
    tag cluster. Expect a tuned yaw/distance offset from "cluster center" to
    "opening center".
- **Field map.** Exact tag poses: take them from the official BIOBUZZ field CAD
  (manual §9.1, ±1 in tolerance) or measure. I did not find a published Limelight
  BIOBUZZ `.fmap`; check limelightvision.io before building one. Tags only have a
  fixed pose while their CELL is up, so a map should use the up-state pose.
- **SDK.** Check whether the current FTC SDK's `AprilTagGameDatabase` already includes
  IDs 30–45 for BIOBUZZ. This project builds against SDK **11.0.0**
  (`build.dependencies.gradle`). An unused SDK 12.0 copy with an empty TeamCode sits in
  `FtcRobotController-12.0/`. Neither is confirmed to be the BIOBUZZ-season SDK.

## Suggested approach

1. **Pipelines.** Pipeline 0 = AprilTag (36h11) with **all 16 IDs (30–45) enabled**,
   because MegaTag2 localization uses every visible cluster and a pipeline ID filter
   would hide the other alliance's tags from it. Filter to our alliance's 8 IDs (red
   30–37, blue 38–45) **in code** for aiming and TIP detection, with the alliance
   chosen at OpMode init.
2. **Yaw aim.** Use `result.getFiducialResults()`, keep our alliance's IDs, and compute
   the cluster-center tx. PD-control the turret motor on tx, with a feedforward from
   robot yaw rate so the turret holds aim while the chassis turns. Clamp to turret soft
   limits, and never command past the hard limits.
3. **Distance.** Prefer the tag's 3D pose (`getTargetPoseCameraSpace()` /
   `getRobotPoseTargetSpace()` per fiducial) over `ty` trigonometry, because the tags
   are seen at steep, oblique angles. Fall back to `ty` with the measured camera height
   and pitch only if 3D pose is noisy.
4. **Flywheel.** Interpolated table of distance → target RPM, tuned on the field and
   clamped to `RPM_CAP` (3500 by default, never above 6000). **Keep separate tables for
   POLLEN (2.8 in) and NECTAR (3.6 in)**: they compress very differently between wheel
   and fixed hood.
5. **Localization.** Odometry first, with periodic Limelight corrections. See the
   full section "Localization (odometry + Limelight)".
6. **Lost target.** Hold the last good turret angle, or aim from odometry pose to the
   known CELL location, until tags return.

## Field coordinates (use everywhere: AUTO, teleop, shot solver)

DECODE autos started at Pose(0,0) relative to the robot. **This year use absolute field
coordinates** so odometry, Limelight pose corrections and target positions all agree,
and the AUTO's final pose carries into teleop.

Pedro-style frame, inches:
- **Origin:** the corner where the **red alliance wall meets the audience wall** (tile
  A1).
- **+x** runs toward the blue wall (columns A→F = x 0→144).
- **+y** runs toward the far wall (rows 1→6 = y 0→144).
- **Heading:** 0 = +x, counter-clockwise positive.

Convert Limelight botpose (FTC field frame, meters, origin at field center) into this
frame in one helper.

**The field is 180° rotationally symmetric, not mirrored.** Red's garden is in the A1
corner and blue's in F6; red's loading zone is on A5 and blue's on F2. So the blue
version of any red pose is:

```
x' = 144 − x,   y' = 144 − y,   heading' = heading + π
```

**Do not mirror only one axis like the DECODE autos did.** Write every AUTO for red and
generate blue with `rotate180(pose)`.

### Key field positions (red side, approximate ±1–2 in)

Confirm all of these from the official BIOBUZZ field CAD (manual §9.1). They are read
off the manual's figures.

| Thing | Approx position (in) | Notes |
|---|---|---|
| Red side of field in AUTO | x 0–72 (columns A–C) | G402: stay on our side in AUTO. Treat x > 72 as a no-go zone, accounting for robot half-width plus any overhang. |
| Red HIVE pivot | x ≈ 59.3, y ≈ 72, pivot 43.95 in up | HIVES are 25.5 in apart center to center, symmetric about x = 72. |
| Red audience-side CELL (tags **34–37**) | x ≈ 59.3, y ≈ 56.6 (**unverified**, see note) | **Up at match start** (holds 3 red NECTAR). |
| Red far-side CELL (tags **30–33**) | x ≈ 59.3, y ≈ 87.4 (**unverified**, see note) | Down at start; comes up after the first red TIP. |
| Up-CELL opening | ~20 in wide × 14 in tall, **53.5–65.6 in** above tiles; aim point ≈ 59.5 in up | The opening tilts 30° up and outward (away from the pivot, along y). Shots from the side it faces should be easiest; shots from behind must clear the higher inner rim. **Verify on the field.** |
| HIVE frame legs | base ≈ 49.5 in (x) × 39 in (y), centered on (72, 72) | Robots fit under the HIVE (its lowest point is ~30.6 in, robot ≤ 29 in), but the **frame legs are obstacles**. Get the exact footprint from CAD for Pedro paths. |
| Red LOADING ZONE (AUTO PARK) | x 0–11, y ≈ 95–118 (on the red wall, row 5) | PARK = robot at least **partly** inside. A robot must **not start** in it (G304.E). |
| Red GARDEN (4 POLLEN) | x 0–23, y 0–2 (along the audience wall from the A1 corner) | POLLEN in a line starting at the corner. |
| Red-side FLOWER 1 (4 POLLEN) | on the far wall, x ≈ 48, y ≈ 144 | Pull POLLEN from the **bottom retrieval opening** only (G418). |
| Red-side FLOWER 2 (4 POLLEN) | on the red wall, x ≈ 0, y ≈ 48 | Same. |
| Blue equivalents | `rotate180()` of the above | Blue's up CELL at start is the **far** one (tags **42–45**); after a TIP it's tags **38–41**. |

**Note on CELL y positions:** 56.6 / 87.4 imply CELLS 30.8 in apart, but the HIVE
geometry above says ~18.8 in (→ y ≈ 62.6 / 81.4). Keep the CELL positions as named
constants in `RobotConstants` and set them from the field CAD.

## Localization (odometry + Limelight)

**The team's plan:** odometry is the main source of position and **direction**
(heading). The Limelight only **adjusts** it every few seconds when it gets a
trustworthy fix. Everything (Pedro paths, shot solver, turret aim, PoseStorage) reads
one pose from one place.

### Ownership: one pose, one owner

- Write a `Localizer` class (or use Pedro's follower as the owner) that is the **only**
  code allowed to read Pinpoint and to change the pose. Everything else calls
  `getPose()` and `getVelocity()`.
- DECODE autos set the pose in two places (`pinpoint.setPosition(...)` **and**
  `follower.setPose(...)`). Pick one. Recommended: **Pedro's follower with its Pinpoint
  localizer** (already configured in `pedroPathing/Constants.java`) in **both** AUTO and
  teleop. Apply corrections with `follower.setPose(...)`. Call `follower.update()` once
  per loop; it updates Pinpoint.
- Pose is in the field frame above (Pedro inches, red/audience corner origin).

### Odometry: the main source

- **Hardware:** goBILDA Pinpoint with 4-bar pods. Encoder directions: forward pod
  FORWARD, strafe pod REVERSED (from DECODE).
- **Pod offsets:** measure and fix them. DECODE disagrees: `MainTeleop` uses (0, 0) mm
  but `Constants.java` uses `forwardPodY(-6)`, `strafePodX(10)` mm. Wrong offsets make
  the position drift every time the robot spins, which matters a lot when the robot
  turns while the turret tracks. Measure from the robot's rotation center to each pod
  and put the result **only** in `Constants.java`.
- **Heading** comes from the Pinpoint's IMU and pods. **The Limelight never overwrites
  heading by default.** MegaTag2 needs our heading as an input, so it can't correct it.
- **INIT:**
  - Call `resetPosAndIMU()` **in INIT** while the robot is still (no motion, so it's
    G304.H-legal).
  - Wait ~0.25 s, then set the start pose from the selected start preset.
  - Show "LOCALIZER READY" on telemetry, and **warn if the robot moved during INIT**
    (IMU disturbed).
- **AUTO → teleop:** store the final pose in `PoseStorage` and start teleop from it.
  Don't reset Pinpoint in teleop INIT.
- **Teleop recovery:** a driver button for "re-seed pose". Push the robot into a known
  corner or wall spot, press the button, and the pose is set to that preset (heading
  included). Use it if the robot gets hit hard and the heading is off.
- **Retune** Pedro's constants (`Constants.java` mass, zero-power accelerations,
  velocities, PIDF) if the robot's weight or drivetrain changed.

### Limelight corrections: every few seconds, only when trustworthy

**Setup:**
- AprilTag pipeline with a **BIOBUZZ field map** containing all 16 tag poses in their
  **up-facing-cell** pose. Only an up-facing cell's tags face the floor, so any visible
  cluster is from an up cell. That holds for both alliances' HIVES, so all 16 IDs are
  usable for localization.
- Limelight's "camera pose in robot space" (rotating turret, later) = the
  **turret-frame** camera pose: forward 0.1165 m, side 0, pitch +37°, and up = lens
  height above the tiles (`CAMERA_HEIGHT_M`, ≈ 0.354 m) so the "robot" origin is the
  turret axis at floor level. Verify sign conventions in the web UI. (Fixed turret,
  now: use the robot-center pose from "TEMPORARY: treat the turret as FIXED".)
- **Every loop**, call `limelight.updateRobotOrientation(toLimelightYawDeg(
  robotHeadingDeg + turretAngleDeg))` for MegaTag2. The camera's yaw is the robot's
  heading plus the turret angle (turret angle = 0 while fixed).
- **Heading frames differ.** Pedro heading is 0 = +x (toward the blue wall), CCW
  positive. The Limelight/FTC field frame may put 0° on a different axis. The
  conversion helper must convert **heading as well as position, in both directions**
  (yaw sent to `updateRobotOrientation`, and botpose read back). A wrong yaw input
  makes every MegaTag2 fix wrong. Check it by parking at a known spot facing +x and
  confirming the Limelight's 3D field view shows the robot facing the blue wall.

**Converting the result to a robot pose:**
- MegaTag2 (`result.getBotpose_MT2()`) returns the **turret frame's** field pose,
  because we gave the camera pose relative to the turret.
- Robot center =
  `turretAxisFieldPos − rotate(robotHeading) · TURRET_AXIS_OFFSET_FROM_ROBOT_CENTER`
  (`TURRET_AXIS_OFFSET`, default (0, 0); see the hardware section).
  **Apply this only when the Limelight camera pose is configured relative to the
  turret axis** (rotating turret). In fixed-turret mode the offset is already in the
  Limelight's camera pose and MegaTag2 returns the robot center, so subtracting it
  again would count it twice.
- Convert from the Limelight/FTC field frame (meters, field-center origin) to our
  Pedro frame with one helper, covering position **and** heading.
- Verify the helper by parking the robot at 3–4 known tile spots and comparing.
- Verify method names on the current SDK: `getBotpose_MT2`, `getBotposeTagCount`,
  `getBotposeAvgDist`, `getStaleness`.

**Gate each frame. Accept it only if all are true:**

| Check | Starting threshold |
|---|---|
| Result valid, MegaTag2 pose not null | — |
| Tags used | **≥ 2** (a full cluster of 4 is best) |
| Average tag distance | < ~2.2 m (beyond that the downward tags are too oblique) |
| Result staleness | < ~100 ms |
| Robot translation speed | < ~15 in/s (motion blur + latency) |
| Robot turn rate | < ~45°/s |
| Turret turn rate | < ~60°/s (the camera rides on it) |
| No HIVE motion | Not within ~2 s of **any** TIP (ours or the opponent's). Watch for a sudden jump in any cluster's apparent position, and reject vision during HIVE swings. |
| Result inside the field | x, y in 0–144 in, not inside the HIVE frame |

**Combining accepted frames:**
- **Latency compensation:** keep ~0.5 s of odometry history (timestamp → pose).
  Compare each accepted vision pose with the odometry pose **at the camera's capture
  time** (now − staleness − pipeline latency), not the current pose.
  `offset = visionPose − odomPoseAtCapture`.
- Collect offsets in a window, about 5–10 accepted frames over up to ~1 s, and take
  the **median** x and y offset. This rejects single bad frames.
- **Apply at most every ~2–3 s** (team plan: "adjust every few seconds"). Use a
  constant `VISION_CORRECTION_PERIOD_S` the team can tune.
- **How much to apply:**
  - Offset < 0.5 in: skip (not worth a pose jump).
  - 0.5–12 in: apply a fraction, `pose += α · offset` with α ≈ 0.5–0.7. This converges
    over a couple of corrections and never snaps.
  - > 12 in: don't apply right away; it could be a bad map, a wrong cell or a
    reflection. Only apply if **3 consecutive** windows agree within ~2 in. That
    handles a real big error after a collision.
- **Heading is not corrected.** Optionally, compute a heading estimate from multi-tag
  MegaTag1 for **telemetry only**, and warn the drivers if it disagrees with odometry
  by > 3° for several seconds (they can then use "re-seed pose").
- **In AUTO:** apply corrections only when the robot is stopped or between Pedro
  paths, since `follower.setPose` mid-path can jerk the path follower. In teleop, apply
  them anytime they pass the gates.

**Telemetry and debugging (Panels):** odometry pose; last accepted vision pose; current
offset; corrections applied; time since last correction; reject counts by reason. Draw
both poses on the Panels field view so the team can see drift and corrections live.

### Separate jobs: localization vs aim correction

Two different uses of the Limelight. Don't merge them:
1. **Localization** (this section): slow, gated, median-filtered pose corrections
   every few seconds.
2. **Aim fine-tuning** (shot solver step 3): fast, per-frame tx correction of the
   turret toward our up cell's cluster center while it's visible.

The shot solver's odometry-based aim works with neither; both make it more accurate.

### Testing

- **Odometry only:** drive a loop around our half and return to the start mark. Record
  drift (x, y, heading). Fix pod offsets until it's small.
- **Spin test:** spin in place 10 times. The position should barely move. If it
  circles, the pod offsets are wrong.
- **Corrections on:** start the robot deliberately ~6 in off from where the code thinks
  it is. Confirm corrections pull it back within a few seconds without overshooting or
  jerking.
- **Bad-data test:** cover the camera, wave a printed tag, and trigger a HIVE tip
  while the robot watches. Confirm the gates reject it all.

## Turret rotation limits (cable wrap protection)

**The team's concern:** the turret must never rotate far enough to wrap or pull out the
wires. That means the Limelight USB plus the flywheel motor's power and encoder cables,
since the 5203 rides on the turret. There is no slip ring, so **the turret must never
make a full turn**. Every command must stay inside a fixed travel range measured from a
known home position.

### Angle tracking

- Keep the turret angle as an **absolute, unwrapped** value in degrees from the
  encoder: `turretDeg = (ticks − homeTicks) / TICKS_PER_TURRET_DEG`. **Never wrap this
  value to ±180.** The wiring cares about total travel from home, not the direction
  it's pointing. +350° and −10° point the same way, but one of them has the cable
  wrapped almost a full turn.
- **Home (0°)** = a fixed physical position, e.g. camera facing robot-forward, with a
  visible alignment mark on the turret and chassis. TODO: the team picks it.
- **Zeroing:**
  - At AUTO INIT the turret is set on its home mark by hand. The code records
    `homeTicks` **without moving anything** (G304.H).
  - Store `homeTicks` (or the current angle) in `PoseStorage` at the end of AUTO.
    **Teleop must not re-zero**, because the turret may not be at home when AUTO ends.
  - If teleop starts without AUTO having run (practice), assume the turret is on its
    mark.
  - Optional: a limit switch or magnet sensor at home, to verify or re-zero during a
    match without guessing.
- **Driver re-zero:** a gamepad combo (e.g. G2 back + start, held 1 s) sets the
  current position as home after the driver lines up the mark manually. This is a
  recovery tool if the encoder slips.

### Limits (constants, tune on the real robot)

| Constant | Starting value | Meaning |
|---|---|---|
| `TURRET_MIN_DEG` / `TURRET_MAX_DEG` | **−150 / +150** until measured | Soft limits. Total travel must stay **< 360°** unless the team confirms the cables allow more. |
| `TURRET_SLOW_ZONE_DEG` | 20 | Within this distance of a limit, scale max power down linearly toward the limit. |
| `TURRET_LIMIT_MARGIN_DEG` | 5 | Setpoints are clamped this far inside the limits, so the PID overshoot never touches them. |

**How to measure the real limits:**
1. Robot on blocks, all cables connected, turret on its home mark.
2. Rotate by hand (power off) each way until a cable gets tight.
3. Record those angles.
4. Subtract a 10–15° safety margin.
5. Write them into the constants.
6. Re-check after any wiring change.

### `Turret.chooseSetpoint(desiredYaw)` (the only way to aim)

The shot solver and vision give a desired direction (−180..180 relative to the robot).
Convert it to a legal absolute setpoint:

1. Candidates: `desiredYaw + 360·k` for k = −1, 0, 1.
2. Keep only candidates inside `[MIN + margin, MAX − margin]`.
3. If **one or more** fit, pick the one **closest to the current turret angle** (least
   travel). If the total range is ≥ 360°, two candidates can both fit; still pick the
   closest. Only "unwind" toward 0 when not shooting and the closer candidate sits in
   a slow zone.
4. If **none** fit (the target is in the dead zone behind the travel range):
   - Clamp to the nearest limit.
   - Set `turretCanReach = false`, so `canShoot` becomes false.
   - Report "TARGET OUT OF TURRET RANGE" on telemetry and LEDs.
   - **In AUTO:** the path planner chooses robot headings at shooting spots so the
     target sits well inside the turret range. Check this offline for every
     shooting spot.
   - **In teleop:** tell the driver (LED) to turn the chassis. An optional assist can
     rotate the chassis just enough to bring the target back in range.
5. Vision tx corrections also go through the clamp. **No code path may set a turret
   setpoint or power directly.**

### Enforcement in the motor layer (last line of defense)

Inside `Turret.update()`, applied to the final power **every loop**, regardless of mode
(auto-aim, manual, re-home, test OpModes):
- If `turretDeg ≥ MAX` and power > 0, or `turretDeg ≤ MIN` and power < 0: **power =
  0**.
- In the slow zone, cap power in the direction of the limit.
- **Stall / jam check:** if power above ~0.3 is applied but the encoder barely moves
  for > 0.3 s, or the motor current (`DcMotorEx.getCurrent`) stays high, cut power and
  flag `TURRET_FAULT`. A jammed turret or a broken encoder cable must not keep driving
  into the wires.
- **Encoder sanity:** if the encoder jumps impossibly between loops, or the angle
  leaves `[MIN − 20, MAX + 20]`, stop the turret and flag a fault until re-zeroed.
  Better a turret that stops aiming than one that spins the wires off.
- Manual turret control (operator stick) goes through exactly the same limits.

### Testing checklist

- With the robot on blocks, command setpoints past each limit and confirm the turret
  stops at the soft limit.
- Drive the robot in circles with auto-aim on, and confirm the turret never chases the
  target around past a limit. It should clamp and report out-of-range.
- Unplug the turret encoder (or simulate a frozen value) and confirm the fault cuts
  power.
- Run AUTO then TELEOP back to back and confirm teleop keeps the same zero.

Also recommended (hardware, not code): a physical hard stop just outside the soft
limits, as a backup if the software ever fails.

## Shoot from anywhere (shot solver, shared by AUTO and teleop)

One class (e.g. `ShotSolver`) runs every loop.

**Inputs:**
- Robot pose and velocity: Pinpoint, corrected by Limelight MegaTag2 when our tags are
  visible.
- Turret angle (encoder).
- **Which of our CELLS is up** (see "TIP tracking").
- Ball type (POLLEN or NECTAR), if the indexer can tell.

**Outputs:**
- Turret angle setpoint.
- Flywheel RPM setpoint.
- A boolean `canShoot`.

**Steps:**

1. **Target point** = field position of our up CELL's opening center (table above,
   per alliance).
2. **Turret yaw (feed-forward from odometry).**
   - Turret-axis field position = robot pose + rotated turret-axis offset (TODO
     offset).
   - `desiredYaw = wrap(atan2(target − turretAxis) − robotHeading)`, then pass it
     through `Turret.chooseSetpoint()` (see "Turret rotation limits"). That function
     never returns an angle outside the soft limits.
3. **Vision correction.** When our cluster is visible, add a small correction from the
   cluster-center tx, rate-limited. Pose corrections are a separate, slower process (see
   "Localization").
   Odometry carries the aim when tags drop out. This is what makes "anywhere" work,
   including spots where the camera can't see the tags (too close, too oblique, or
   blocked by robots).
4. **Distance** = horizontal distance from the turret axis to the target point.
5. **RPM** = interpolate `rpmTable[ballType](distance)`, then clamp to `RPM_CAP` (3500
   by default, never above 6000).
6. **Shoot while moving (v2, after stationary shots work).** Aim at
   `target − robotVelocity × timeOfFlight(distance)`, with time of flight from a small
   tuned table. Start with the robot stopped or slow.
7. **`canShoot`** requires all of:
   - A target is known.
   - `turretCanReach` is true (target inside the turret's soft limits) and there is no
     `TURRET_FAULT`.
   - Turret error < tolerance. The opening is ~20 in wide, so allow about
     `atan(6 in / distance)`.
   - Measured RPM within ±100–150 of the setpoint for a few loops (DECODE used ±120–150).
   - Not inside the post-TIP settle window.
   - Not in a no-shoot zone (too close under the HIVE, and anything testing shows
     misses).
   - Holding at least 1 ball.
   - Match period allows shooting: not in the AUTO→TELEOP transition (G403).

   The feeder (DECODE's stopper servos + feeder motor) only runs while `canShoot` is
   true. **This is how we satisfy G417**: we only launch at our own up-facing CELL.

**Building `rpmTable`:**
- Test from a grid of spots: every ~12–18 in across our half first, then the rest of
  the field for teleop. Log distance, RPM and hit/miss (Panels is already a
  dependency).
- Keep POLLEN and NECTAR tables separate.
- Expect the table to be smooth and increasing with distance. Fit it, don't hand-pick
  every point.

**Is 3500 RPM enough? Rough physics, verify by testing.**
- Ball leaves ≈ 0.4 m up; target ≈ 1.51 m up, so it must rise ≈ 1.1 m.
- Minimum launch speed (at the ideal angle):
  - ≈ **6.1 m/s** at 2.4 m (farthest own-half shots).
  - ≈ **6.6 m/s** at 3.1 m (cross-field in teleop).
- At 3500 motor RPM, with 1:1 to the 73.4 mm wheels (unconfirmed), the wheel surface
  moves ≈ 13.4 m/s. Hooded shooters typically launch at roughly half of that, so
  ≈ 6–7 m/s.
- So 3500 should cover our half but may be **marginal for the longest shots**,
  especially since the fixed hood won't be at the ideal angle everywhere.
- If far shots fall short in testing, raise `RPM_CAP` instead of restructuring code.
  The motor can do 6000.

### TIP tracking (needed to know which CELL to shoot)

- **Start state:** red's up CELL = audience CELL (34–37); blue's = far CELL (42–45).
  This is the setup in manual §10.3.1. Let the drive team flip it in INIT if a field
  is set up differently.
- **Detect a TIP:** our visible cluster changes from one CELL's IDs to the other's.
  For red that's 34–37 → 30–33 or back; confirm over ~3–5 frames. Toggle `upCell`.
- **TIP threshold (from the team): 8 POLLEN** launched into the up CELL tips the HIVE.
  Make it a constant `BALLS_TO_TIP = 8`. NECTAR's weight is unknown, so count one
  NECTAR as 1 until tested.
  - Our shots alone can't track the true fill level, because our partner shoots into
    the same CELL. Use the count as an estimate.
  - Show "~N more to tip" on telemetry for the drivers.
- **Fallback:** if tags aren't visible, count our shots into the current CELL and mark
  "tip likely" at `BALLS_TO_TIP`. Confirm with vision as soon as possible. Reset the
  count on every confirmed TIP.
- **After a TIP:**
  - Pause shooting ~1–1.5 s (tuned) while the HIVE swings and spills. Shots at the
    moving / down-facing CELL can stop the TIP from registering (manual §10.5.1).
  - Then retarget the new up CELL.
  - Count TIPS in telemetry. The POLLINATOR RPs need 4 and 7 TIPS per match.
- Spilled balls land around the HIVE. **Never position the robot to catch them**
  (G409); collect them only after they've hit the floor.

## Autonomous (AUTO): 30 s

### What scores in AUTO (per robot unless noted)

| Item | Points | How |
|---|---|---|
| LEAVE | 3 | Not touching the perimeter wall at the end of AUTO. |
| AUTO PARK | 5 | At least partly in our LOADING ZONE at the end of AUTO. |
| HIVE TIP | 20 (alliance) | Any TIP completed before TELEOP starts counts as AUTO. |
| Balls left in the up CELL | 2 each (alliance) | Counted at **match end** only, not in AUTO. |
| FLOWER / GARDEN | — | FLOWER scoring isn't allowed until the last 60 s, and GARDEN counts only at the end. |

LEAVE + both PARKs feed the **SWARM RP** (≥ 16 combined LEAVE + PARK points per
alliance across the match). TIPS feed **POLLINATOR RPs** (4 and 7). Avg AUTO points is
the 4th ranking tiebreaker.

### Starting rules (G304) the code must respect

- **Start position.** Fully on our side (columns A–C for red), **touching a perimeter
  wall**, not in the LOADING ZONE, not touching or inside a FLOWER, inside the 18 in
  cube.
- **Pre-loads.** Touching **exactly 4 pre-load POLLEN**, which we'll hold in the robot.
- **INIT.** After INIT finishes the robot must be **completely still**:
  - No turret seeking, no flywheel, no intake.
  - Holding servos/motors at a fixed position is fine (R103).
  - Pinpoint reset, IMU reset, Limelight start and pipeline select are fine (no
    motion).
  - **Do the Pinpoint reset in INIT.** DECODE's `BlueFar` called `resetPosAndIMU()`
    *after* start and waited 0.3 s. That wastes AUTO time.
- **The AUTO OpMode must have the 30 s timer enabled (G305).** Use
  `@Autonomous(preselectTeleOp = "...")` so TELEOP is queued automatically.

### Rules during AUTO

- **G402: stay on our half.** Don't touch balls or robots on the blue side. Geofence
  all paths to x < 72 minus robot half-width. Our turret can still shoot into our HIVE
  from our side.
- **G407: max 4 balls controlled.** The pre-load is already 4, so the intake must not
  take a 5th; stop intaking at 4.
- **G408:** don't pick up opponent NECTAR.
- **G417:** shoot only at our up CELL. Never at the HIVE's outside, never at blue's HIVE.
- **G409:** don't sit under the HIVE waiting for spills.
- **G418:** FLOWER POLLEN may only be taken out through the **bottom** retrieval
  opening. No NECTAR into FLOWERS in AUTO (G410 — MAJOR FOUL per NECTAR).
- **G403:** when AUTO ends, nothing may move under power during the 8 s transition.
  The SDK stops the AUTO OpMode at 30 s. TELEOP INIT must not move anything; if it
  would, drivers wait for TELEOP to start before pressing INIT.

### INIT selection menu (gamepad in `init_loop`)

Let the drive team choose before the match, shown on telemetry and with confirmation
LEDs:
- **Alliance:** red / blue. This drives `rotate180()`, tag IDs and the target CELL.
- **Start position:** at least 3 presets, see below.
- **Route:** e.g. "Preload only + park", "Preload + flower", "Preload + garden",
  "Preload + spills".
- **Start delay:** 0–10 s in 0.5 s steps, so we don't collide with our partner.
- **Initial up CELL:** default per manual; flip if the field was set up differently.

Also store the final AUTO pose and turret encoder offset in a static field (e.g.
`PoseStorage`) so teleop starts with the right field pose. **Don't re-zero the turret
encoder in teleop.**

### Start positions (red, approximate; blue = `rotate180`)

All must touch a wall and avoid the LOADING ZONE and FLOWERS. Tune the exact numbers on
a real field.

| Name | Where | Why |
|---|---|---|
| `AUDIENCE` | against the audience wall, x ≈ 30–60, y ≈ robot half-length | Closest to our up CELL at start (audience CELL at y ≈ 56.6), ~3–4 ft shot. Near the GARDEN. |
| `ALLIANCE_WALL` | against the red wall, x ≈ half-width, y ≈ 60–88 (between FLOWER 2 at y≈48 and the LOADING ZONE at y≈95) | Short drive to the LOADING ZONE for the park. Clear view of the HIVE. |
| `FAR` | against the far wall, x ≈ 10–36 (avoid FLOWER 1 at x≈48) | Leaves the near side to our partner. Shoots from behind the up CELL, so check accuracy. |

### Baseline AUTO plan (write this first)

The flywheel stays spinning for the whole AUTO once started. **Do not brake it between
volleys** like DECODE did (spin-up timeouts of up to 5 s cost too much in 30 s).

| Time (s) | Action |
|---|---|
| 0.0 | START. Begin flywheel spin-up to the solver RPM. Turret aims from odometry immediately. Wait any configured delay. |
| 0–2 | Drive off the wall toward a shooting spot (LEAVE). The shot solver means we can **fire as soon as `canShoot` is true**, even while still moving slowly. |
| ~1–5 | Fire the 4 pre-load POLLEN. Space shots so RPM recovers: wait for the ready band between shots. Count shots with an RPM-dip detector or beam-break, not a fixed timer (DECODE's `SHOOT_DURATION_SEC`). |
| ~5 | Watch for a TIP (our cluster switches IDs). On a TIP: pause ~1–1.5 s, retarget the new up CELL, add 20 to the telemetry estimate. **8 POLLEN tips a HIVE, so our 4 pre-loads + our partner's 4 = an AUTO TIP (20 pts)** if both robots score all their pre-loads. Agree on this with the partner in queue. If our partner has no AUTO, we need 4 more POLLEN for the first TIP. |
| ~5–22 | **Cycle** (route selected in INIT): drive to a POLLEN source on our side, intake up to 4, drive back toward a good shooting spot, and fire whenever `canShoot`. Sources, in order to try: (1) the FLOWER retrieval openings on our side (4 POLLEN each, bottom only); (2) our GARDEN line (4 POLLEN; costs 1 end-of-match point each if left in place, a TIP is worth far more); (3) spilled balls around the HIVE after a TIP, once they've landed. |
| **~25 at the latest** | Start the drive to the red LOADING ZONE, wherever we are. Pick the deadline from measured drive times plus a margin. The turret may keep shooting on the way if `canShoot`. |
| < 30 | Be at least partly in the LOADING ZONE (AUTO PARK +5) and **not touching the wall** (LEAVE +3). The zone is against the red wall, so park poses must keep the robot a couple of inches off it while still overlapping the zone. Stop all mechanisms a moment before 30 s. |

Notes:
- **Time budget.** Use `opModeTimer` everywhere and give every state a timeout that
  falls through to "go park". A stuck intake must never cost the 5-point park.
- **Partner coordination.** Two robots share our half and our LOADING ZONE
  (23 × 11 in, "partly in" is enough). Offer park-spot variants (upper/lower half of
  the zone) and the start delay. Coordinate routes in queue: one robot takes FLOWER 1
  (far wall), the other the GARDEN / FLOWER 2.
- **Pedro paths** stay in red coordinates. Build them in `buildPaths(alliance)` by
  mapping every pose and control point through `rotate180()` for blue. This fixes the
  DECODE `RedMain` control-point bug by construction. Avoid the HIVE frame legs and the
  x = 72 line.
- **Localization.** Seed the start pose from the selected preset, then follow
  "Localization (odometry + Limelight)". In AUTO, apply vision corrections at shooting
  stops or between paths, not in the middle of a Pedro path, unless the correction is
  tiny.
- **Telemetry (Panels).** State, timer, pose, turret setpoint/actual, RPM
  setpoint/actual, `canShoot` and why not, up CELL, TIP count, balls held, estimated
  AUTO points.

### Stretch goals (after the baseline is reliable)

- Shoot while moving through the cycle paths, using the velocity lead in the solver.
- A second cycle if the first finishes before ~15 s.
- Limelight color pipeline (POLLEN = yellow) to find spilled POLLEN, switching
  pipelines away from AprilTags only while collecting.
- A second TIP in AUTO: after the first TIP the new up CELL needs 8 more. Between both
  robots that's two full loads of 4, so it needs fast cycles.

### Open questions for the team (AUTO)

- Does NECTAR count the same as POLLEN toward the 8-ball TIP? (Test.)
- Can the human player enter the unlocked NECTAR during AUTO after an AUTO TIP?
  G426/G427 don't say it's TELEOP-only. **Ask the official Q&A before relying on it.**
- Real drive + intake times, to set the park deadline.
- Which start positions and routes our usual partners prefer.

## Unknown values: placeholders + calibration OpModes

The team doesn't know several hardware values yet. **Don't block on them.** Put every
one in a single `RobotConstants` class (Panels `@Configurable` where it helps live
tuning), give it a safe placeholder, and write small test OpModes that measure it.
Show a telemetry warning for each value still at its placeholder.

| Value | Placeholder | How it gets measured |
|---|---|---|
| Turret motor name / port | `"turret"` | Team sets it in the robot config. |
| Turret direction | positive power = counter-clockwise from above = +deg | `TurretCalibration`: tap power, check that the angle increases. Flip the motor direction if not. |
| `TURRET_TICKS_PER_DEG` | none; **turret auto-aim disabled until set** (manual turret only, still limited) | `TurretCalibration`: motor set to FLOAT, no power. The driver turns the turret by hand from the home mark to a 90° mark (e.g. camera facing robot-left); the OpMode shows ticks/deg. Repeat both ways and average. |
| `TURRET_MIN_DEG` / `TURRET_MAX_DEG` | −150 / +150 | `TurretCalibration` "limit finder": turn by hand until a cable is tight each way. The OpMode records it and suggests limits minus a 15° margin. |
| `TURRET_AXIS_OFFSET` | (0, 0) in | Tape measure (see the hardware section). |
| `CAMERA_HEIGHT_M` | 0.354 | Tape measure to the lens center. |
| Flywheel motor→wheel ratio | **not needed** | RPM tables are in **motor RPM** (encoder at the motor, DECODE used 28 ticks/rev). Tuning is distance → motor RPM directly, so the ratio never enters the code. It only matters for the physics estimate in "Is 3500 RPM enough?". |
| Pinpoint pod offsets | the `Constants.java` values (`forwardPodY -6`, `strafePodX 10` mm); delete the `(0, 0)` override from teleop code | Measure with a ruler, then check with the 10-spin test (see "Localization → Testing"). Pedro's `Tuning.java` has localization tests. |
| `BALLS_TO_TIP` | **8** (team) | Known. Test whether NECTAR counts the same. |
| Field positions (HIVE, CELLS, LOADING ZONE, FLOWERS) | the approximate table in "Field coordinates" | Official field CAD, or measure on a practice field. |
| RPM tables (POLLEN, NECTAR) | a straight line from ~2000 RPM at 1 ft to 3500 at max range (placeholder only) | `ShooterTuning` OpMode: the driver parks anywhere and the solver shows distance. Adjust RPM with the d-pad until shots go in, then press a button to save the (distance, RPM) point to a file on the Control Hub (or log it to telemetry/Panels). Fit the table from those points. |

**Test OpModes to write** (TeleOp group `"Calibration"`):
- `TurretCalibration`: ticks/deg, direction, limit finder, home-mark check.
- `ShooterTuning`: RPM tables.
- `LocalizationTest`: odometry pose vs Limelight pose, correction log, Panels field
  drawing.
- `VisionCheck`: which tag IDs are visible, cluster center tx, MegaTag2 pose, reject
  reasons.

## Rules that constrain the code

Details are in BIOBUZZ_RULES.md.

- **Nothing moves after INIT (G304.H)** until the match starts. The turret must not seek
  and the flywheel must not spin during init. Holding position to stay in starting
  configuration is allowed (R103).
- **No powered motion in the 8 s AUTO→TELEOP transition (G403).** If INIT twitches
  actuators, the drivers must wait for TELEOP before pressing INIT.
- **Motionless after TELEOP ends (G404).**
- **Only LAUNCH into the upward-facing CELL (G417).** Deliberately hitting a CELL's
  bottom, sides or top, or messing with the opponent's TIP, is penalized. **Gate the
  feeder:** require a valid target from our alliance's IDs, aim error under a
  threshold, and flywheel at speed. Pause shooting while our HIVE is tipping, because
  shots into the downward-facing CELL can stop the TIP from registering.
- **Max 4 scoring elements controlled at once (G407).** Indexer/intake code should stop
  at 4.
- **Don't control opponent NECTAR (G408).** Color sorting should reject it fast.
- **NECTAR only into FLOWERS in the last 60 s (G410).** That is MAJOR FOUL per NECTAR,
  so a match timer in code helps.
- **Size limit 18 × 24 × 29 in once expanded (R105).** It must be enforced
  **mechanically**; software limits do not count. Check that the turret plus anything
  hanging off it stays inside the footprint at every rotation angle.
- **Limelight 3A is a legal, reprogrammable vision coprocessor (R702, R708).** The
  Limelight 3G is not allowed. USB is only for vision devices and hubs (R707).
- **No mimicking field AprilTags (G302.E).** Don't put 36h11 tags on the robot.

## Reuse from DECODE

Our DECODE (2025-26) code is this project's
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. It is a standard
FtcRobotController TeamCode project:
- SDK 11.0.0
- Pedro Pathing 2.0.4 (`com.pedropathing:ftc`)
- Panels 1.0.9 (`com.bylazar:fullpanels`)

The newest and best file is `MainTeleop.java` (Feb 20, 2026). The autos are `BlueFar`,
`RedFar`, `BlueMain` and `RedMain`, all Pedro state machines. Files prefixed `OLD` are
superseded.

**Important difference:** DECODE had **no turret**. It aimed by **rotating the whole
chassis** on Limelight `tx` (`computeLimelightAimRotate()` feeds `mecanumDrive`'s
rotate). BIOBUZZ aims with the turret motor, so all aim gains must be retuned for a
turret.

### Reuse (copy the logic and retune the numbers)

| What | Where in `MainTeleop.java` | Notes |
|---|---|---|
| Flywheel velocity control | shooter PIDF block (`kP 0.015`, `kD 0.0001`, `kF 0.000422143`, anti-windup, clamp) | Power-based custom PIDF with `SHOOTER_TICKS_PER_REV = 28`, `MAX_RPM = 6000`. That suggests a 1:1 (6000 RPM) Yellow Jacket with the encoder read at the motor. The new turret's flywheel motor is also a 5203; confirm its ratio. **Re-tune kF** for the new wheels and hood. |
| Idle spin + ramp | `IDLE_RPM 1200`, `rampTowards()` | Open-loop idle with a ramp to avoid brownouts. Good to keep. |
| "Ready" band | `shooterReady`: within ±150 RPM | Reuse as one of the shot-gate conditions (G417). |
| Limelight filtering | `updateLimelightOriginalFiltered()` | EMA filter (`LL_EMA_ALPHA 0.35`) on tx/ty/distance. Keep the idea. |
| Aim lock | deadband 0.5°, `TAG_LOCK_STABLE_CYCLES 5`, fine/coarse max power | Same pattern works for the turret. |
| Distance fusion | `fuseDistanceMeters()`: 0.85 Limelight + 0.15 odometry | Keep, but odometry distance must be to the real CELL position (see below). |
| Odometry fallback aim | `computeAimToPointRotate()` | Becomes "turret angle = bearing to CELL − robot heading" when tags are lost. |
| Feed gate | stoppers (`leftStop`/`rightStop` servos) + LEDs (red/orange/yellow/green) | Natural place to enforce "only feed when aimed + at speed + valid target". |
| Pedro + Pinpoint setup | `pedroPathing/Constants.java` | Mecanum and Pinpoint constants are reusable if the drivetrain is unchanged. |

Hardware map names used last year: `frontLeft`, `frontRight`, `backLeft`, `backRight`,
`intake`, `feeder`, `shooter`, `ledLeft`, `ledRight`, `leftStop`, `rightStop`,
`pinpoint`, `limelight`.

### Must change for BIOBUZZ

1. **Target selection.** The old code uses `result.getTx()` (Limelight's primary
   target, *any* tag) and takes the ID from `fiducials.get(0)`. BIOBUZZ has 16 tags in four
   clusters, and the opponent's are often in view too. Go through `getFiducialResults()`, keep
   only our alliance's IDs (red 30–37 / blue 38–45), and aim at the **cluster center**.
2. **Tag lock.** `LIMELIGHT_LOCK_TO_TAG_ID` locks onto one tag ID, but BIOBUZZ targets
   change after every TIP (our other CELL comes up). Lock to "our alliance's currently
   visible cluster", and re-acquire after a tip instead of dropping aim.
3. **Aim bias.** The far-shot bias is keyed to tags 20 (blue goal) and 24 (red goal).
   Replace it with a per-CELL or per-distance offset table.
4. **Distance from ty.** The old constants are `LIMELIGHT_HEIGHT_M 0.50`,
   `TARGET_HEIGHT_M 0.90` and `LIMELIGHT_PITCH_DEG 20`. New:
   - Pitch is **37°**.
   - Camera height ≈ **0.354 m** (estimated, measure on the robot).
   - Target is the downward-facing cluster, roughly 1.1–1.3 m up (measure).
   - tx/ty are now **relative to the turret**, not the chassis.
   - Prefer the 3D fiducial pose.
5. **RPM model.** The old model is sqrt-of-distance around a 1.0 m / 2400 RPM reference,
   plus a fixed 3050 RPM beyond 1.75 m. Refit from scratch for the new shooter, with
   separate POLLEN and NECTAR tables.
6. **Odometry target.** The old odometry "goal" is (0,0) plus a fixed (250, 70) mm
   offset, and autos start at Pose(0,0). Use **field coordinates** this year so
   Limelight pose corrections and known CELL positions mean the same thing in auto and
   teleop.
7. **Red/blue mirroring.** Red autos were hand-copied from blue. **Bug:** in
   `RedMain.java` the Bezier control points `shootToBarControl` and
   `barToShootControl` still use blue's values (Y +63.4, headings −49° / −22.18°),
   while every other red pose has negated Y. That curve swings toward the wrong side.
   Write one auto for red and convert it with a `rotate180(pose)` helper instead of
   duplicating files (the field is rotationally symmetric; never mirror one axis).
8. **Pinpoint offsets disagree.** `MainTeleop` calls `pinpoint.setOffsets(0, 0, MM)`,
   but `Constants.java` uses `forwardPodY(-6)` / `strafePodX(10)` mm. Pick the measured
   values and use them everywhere.
9. **Structure.** Each OpMode is a ~800–1100-line file with the shooter, Limelight and
   LED code copy-pasted (20+ near-duplicate files). Pull out shared `Shooter`, `Turret`,
   `Vision` and `Leds` classes so tuning is done once.
10. **SDK.** Move from SDK 11.0.0 to this season's SDK and check its AprilTag library
    for IDs 30–45.

DECODE tags for reference: goals 20 (blue) and 24 (red), confirmed in `MainTeleop.java`.
DECODE artifacts were 5 in; BIOBUZZ POLLEN is 2.8 in and NECTAR 3.6 in.
