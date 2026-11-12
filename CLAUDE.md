# FTC robot code: BIOBUZZ 2026-27 season

FtcRobotController project (SDK 11.0.0, Pedro Pathing 2.0.4, Panels 1.0.9). The code in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` is **last season's DECODE
code**. Reuse its patterns but write new BIOBUZZ code. Files prefixed `OLD` are dead.
`FtcRobotController-12.0/` is an unused SDK copy; ignore it.

## BIOBUZZ code location

New code lives in `TeamCode/.../teamcode/biobuzz/`: shared classes at the top level
(`RobotConstants`, `Robot`, `Localizer`, `Vision`, `ShotSolver`,
`Aimer`/`ChassisAimer`/`TurretAimer`, `Shooter`, `Feeder`, `Turret`, `TipTracker`,
`HeadingHold`, `PidfStore`, `Field`, `PoseStorage`), `auto/` (engine `BiobuzzAutoBase` +
one OpMode per route, poses in `AutoPoses`), `teleop/BiobuzzTeleOp`, `calibration/`
(Shooter Tuning, Localization Test, Vision Check, Turret Calibration, Turret PIDF Tuner,
Motor Test, PIDF Tuner).

Off-robot check of the turret lock (limits, flip, tag fallbacks, latency) against a
simulated motor and camera: `bash TeamCode/src/test/run-turret-sim.sh`. Run it after
changing `Turret`, `Vision.centerOf` or `ShotSolver.aimDirectionDeg`.

Tuned PIDF gains saved by PIDF Tuner live in `/sdcard/FIRST/biobuzz_pidf.txt` on the Control
Hub and OVERRIDE `RobotConstants` while that file exists.

## Read on demand (don't load unless the task needs it)

- **`docs/biobuzz/UNDERHAND_BRIEF.md`: the CURRENT design.** It covers the underhand
  turret, camera pose, ±225° + 360° flip, controls, 4/2-tag aiming, PIDF tuner and
  distance→RPM. It overrides the hardware and turret parts of the older brief.
- `docs/biobuzz/TURRET_LIMELIGHT_BRIEF.md`: older hooded design (geometry superseded);
  still the reference for field coordinates, localization, the shot solver, TIP
  tracking, the AUTO plan and rules. Original description: turret and Limelight
  geometry from CAD,
  the shot solver, the full **AUTO plan**, field coordinates, what to reuse or change
  from DECODE.
- `docs/biobuzz/BIOBUZZ_RULES.md`: full game rules summary (manual TU07).

## Turret: rotating design now specified

The team's underhand design has a **rotating turret** (`UNDERHAND_BRIEF.md`):
- goBILDA 312 RPM motor + goBILDA turret.
- ±225° travel with a 360° flip.
- G2 left-stick press toggles auto/manual; left stick x for manual.
- Limelight PIDF aim at the cluster center (4 tags, else 2).

`TurretAimer` implements that brief (written, not yet run on the robot). Keep
`TURRET_FIXED = true` until the turret ticks/deg is calibrated, then switch it to false,
re-enter the Limelight camera pose relative to the turret axis (`Vision.java` header)
and tune the gains with Turret PIDF Tuner.

### Previous temporary mode (still the default until calibrated)

`RobotConstants.TURRET_FIXED = true`. Treat the shooter + Limelight as bolted to the
chassis facing forward, and **aim by turning the whole robot** (DECODE-style chassis
aim, driver keeps translation).
- Turret motor optional and never powered.
- Turret limits and calibration are inactive.
- MegaTag2 yaw = robot heading (converted to the Limelight field frame).
- Limelight camera pose is set relative to the robot center on the floor (offset
  already included), so don't subtract `TURRET_AXIS_OFFSET` from MegaTag2 again.
- AUTO paths end facing the target.

Aiming sits behind the `Aimer` interface (`ChassisAimer` while fixed, `TurretAimer` when
rotating), picked by the flag. Don't delete either.
Details: brief, section "TEMPORARY: treat the turret as FIXED".

## Must-know facts

- **New robot: the underhand design** (Onshape "underhand maybe").
  - A turret on a ring bearing, one shooter wheel (Ø97.8 mm) on a goBILDA 5203; the
    wheel isn't placed in CAD yet.
  - The Limelight 3A sits on the **LL Mount (Part 1 <4>) at the back of the turret**,
    on the centerline, facing CAD +Y.
  - Camera numbers: **pitched +20°**, ≈ 0.094 m from the yaw axis, lens ≈ **0.325 m**
    above the floor (estimate). Robot + turret ≈ 16 in tall.
  - **The shot leaves in the camera's direction** (confirmed), so tx = 0 on the cluster
    center = aimed.
- **Flywheel.** Hard max 6000 RPM; clamp to `RPM_CAP = 3500` by default. Use separate
  RPM tables for POLLEN (2.8 in ball) and NECTAR (3.6 in). All RPM numbers are tunable
  constants; the team will edit them.
- **Turret rotation limits (wiring safety, non-negotiable).** No slip ring: the
  Limelight USB and the flywheel motor's cables cross the turret joint.
  - Track the turret angle **unwrapped** from a home mark, and never wrap it to ±180.
  - Limits are **±225°** from the start position (team spec). Past a limit while
    tracking, **flip 360°** to the equivalent angle (no shooting during the flip).
    Manual mode never flips. This replaces the old ±150° / < 360° rule.
  - All aiming goes through `Turret.chooseSetpoint()`. Out-of-range targets clamp and
    block shooting.
  - `Turret.update()` zeroes power past the limits in every mode, slows near them, and
    cuts power on a stall or encoder fault.
  - Teleop must not re-zero after AUTO (use `PoseStorage`).
  - Details: brief, section "Turret rotation limits".
- **Aiming.** DECODE aimed by rotating the chassis. BIOBUZZ aims with the turret, via
  one shared `ShotSolver`: odometry feed-forward plus Limelight tx correction. It
  must shoot from anywhere.
- **AprilTags** (36h11, 3.25 in, clusters of 4) sit on the **underside of HIVE cells,
  facing down**, and move when the HIVE tips:
  - red 30–33 (far cell), 34–37 (audience cell)
  - blue 38–41 (audience cell), 42–45 (far cell)

  Shoot only at our alliance's **up-facing** cell; its cluster is the visible one. At
  the start red's up cell is the audience cell and blue's is the far cell.
- **Localization.**
  - **Odometry (Pinpoint via Pedro's follower) is the main pose and heading source**,
    with one owner for the pose.
  - The Limelight MegaTag2 corrects **x/y only, every few seconds**, using gated,
    latency-compensated, median-filtered, partial corrections. It never corrects
    heading.
  - Reject vision while moving or turning fast, during HIVE tips, with < 2 tags, or
    from far away. Apply corrections in AUTO only between paths.
  - The camera is on the turret: MegaTag2 yaw input = robot heading + turret angle.
  - Pedro and the Limelight field frame may differ in heading zero. One helper
    converts position **and** heading, both directions.
  - The AprilTag pipeline keeps all 16 IDs (MegaTag2 uses them all). Filter to our
    alliance's IDs in code, for aiming and TIP detection only.
  - Pinpoint pod offsets disagree in the DECODE code and must be measured.
  - Details: brief, section "Localization (odometry + Limelight)".
- **Field frame.** Pedro inches, origin at the red-wall/audience corner, +x toward the
  blue wall, +y toward the far wall. The field is **180° rotationally symmetric**:
  `blue = (144−x, 144−y, h+π)`. Never mirror just one axis.
- **Rules the code must enforce:**
  - Nothing moves after INIT until START (G304.H).
  - No powered motion in the 8 s AUTO→TELEOP transition (G403) or after the match
    ends (G404).
  - Hold at most 4 balls (G407).
  - Stay on our half in AUTO (G402; red x < 72).
  - Launch only into our up cell (G417).
  - No NECTAR into FLOWERS before 1:00 left (G410).
  - The size limit 18×24×29 in must be mechanical, not software (R105).
- **Hardware names:** `frontLeft`, `frontRight`, `backLeft`, `backRight`, `intake`,
  `feeder`, `shooter`, `turret`, `pinpoint`, `limelight`. **No servos on this robot**
  (no stoppers, no servo-driven LEDs): the feeder motor gates balls into the flywheel,
  so it must stay off while intaking.
- **TIP threshold:** **8 POLLEN** tips a HIVE (`BALLS_TO_TIP = 8`). Both robots'
  4 pre-loads = an AUTO TIP.
- **Unknown values:** don't block on them, and don't invent numbers.
  - Use named placeholders in `RobotConstants`: turret ticks/deg (auto-aim disabled
    until set; 312 RPM motor = 537.7 counts per output rev × unknown turret ratio),
    turret limits ±225°, `TURRET_AXIS_OFFSET` (0, 0), camera height 0.325 m, Pinpoint offsets from `Constants.java`, CELL field positions (the
    brief's CELL y values conflict with its 18.8 in spacing; confirm from field CAD).
  - Write calibration OpModes to measure them (brief: "Unknown values").
  - RPM is always motor-encoder RPM, so the flywheel gear ratio isn't needed.

## Code style

- Extract shared classes (`Shooter`, `Turret`, `Vision`, `ShotSolver`, `PoseStorage`)
  instead of copy-pasting 1000-line OpModes like DECODE did.
- One AUTO per route, built for red and converted to blue with `rotate180()`.
