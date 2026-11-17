# Graph Report - FTCSDK  (2026-11-11)

## Corpus Check
- 64 files · ~82,117 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 12 file(s) not represented in the graph (top: (none) 4, .xml 3, .jar 2)

## Summary
- 1232 nodes · 2969 edges · 81 communities (34 shown, 47 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 289 edges (avg confidence: 0.84)
- Token cost: 144,061 input · 0 output

## Community Hubs (Navigation)
- SDK and Library Imports
- Turret Control and Tuner
- DECODE BlueFar Auto
- DECODE RedFar Auto
- Shooter, Feeder and RPM
- DECODE AutoTest
- DECODE BlueMain Auto
- DECODE RedMain Auto
- Localization and Vision Gating
- DECODE BlueMulti Auto
- DECODE RedMulti Auto
- Limelight Tag Clusters
- DECODE Intake Auto Test
- DECODE MainTeleop
- DECODE NewTestTeleop
- TIP Tracking and Up Cell
- Field Geometry
- Shot Solver and AUTO Steps
- BIOBUZZ AUTO Engine
- Heading Hold and Saved PIDF
- Pedro Tuners (Drive/Heading)
- Robot Hub
- DECODE 2-Player Limelight TeleOp
- Pedro Tuning Telemetry
- Shared Classes and Pose Handoff
- BIOBUZZ Game Rules
- AUTO Route Builders
- DECODE Testing OpMode
- Auto Hold Still
- Turret Briefs and Cable Limits
- AUTO Poses
- Constants and Placeholders
- Localization Test
- Pedro Forward/Lateral Tuners
- Turret Aimer and Fixed Flag
- Pedro Path Test Shapes
- Preload Routes and Red/Blue Rotation
- Project Guide (CLAUDE.md)
- Calibration Imports and Ball Type
- AUTO Plan and Field Coordinates
- Pedro Zero-Power Tuners
- Panels Field Drawing
- Alliance and Tag Ownership
- Turret Calibration
- FieldDraw Helper
- BIOBUZZ TeleOp
- Aimer Interface
- Vision Check
- Distance and DECODE Reuse
- 360° Flip Logic
- Shooter Tuning
- Temporary Fixed-Turret Mode
- Auto Preload Flower
- Chassis Aimer
- Underhand Camera Pose
- Turret Tuner Modes
- Pedro Line Test
- Pedro Localization Test
- Forward Velocity Tuner
- Forward Zero-Power Tuner
- Lateral Velocity Tuner
- run-turret-sim.sh
- BIOBUZZ Game Overview
- G304 Robot Placement (motion
- Underhand CAD is mid-build (
- G402 No AUTO Interference (o
- R105 Expansion Limit 18x24x2
- R503 Max 8 Motors + 8 Servos
- Hooded shooter hardware (Tur
- New Ramp (~25° slope toward 
- Shooter 5203 motor on front 
- Turret bearing yaw axis at C

## God Nodes (most connected - your core abstractions)
1. `Turret` - 54 edges
2. `BiobuzzAutoBase` - 52 edges
3. `Robot` - 44 edges
4. `Alliance` - 41 edges
5. `Localizer` - 39 edges
6. `BlueMulti` - 38 edges
7. `RedMulti` - 38 edges
8. `IntakeAutoTest` - 35 edges
9. `Vision` - 34 edges
10. `BlueFar` - 32 edges

## Surprising Connections (you probably didn't know these)
- `During flip: canShoot false, FLIPPING` --conceptually_related_to--> `Feeder`  [INFERRED]
  docs/biobuzz/UNDERHAND_BRIEF.md → TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/Feeder.java
- `Limelight<->Pedro frame helper (position and heading)` --conceptually_related_to--> `Field`  [INFERRED]
  docs/biobuzz/TURRET_LIMELIGHT_BRIEF.md → TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/Field.java
- `TURRET_AXIS_OFFSET (default 0,0)` --conceptually_related_to--> `RobotConstants`  [INFERRED]
  docs/biobuzz/TURRET_LIMELIGHT_BRIEF.md → TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/RobotConstants.java
- `AIM_OFFSET_DEG (cluster vs opening center)` --conceptually_related_to--> `RobotConstants`  [INFERRED]
  docs/biobuzz/UNDERHAND_BRIEF.md → TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/RobotConstants.java
- `Flywheel RPM_CAP = 3500 (hard max 6000)` --conceptually_related_to--> `Shooter`  [INFERRED]
  CLAUDE.md → TeamCode/src/main/java/org/firstinspires/ftc/teamcode/biobuzz/Shooter.java

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Turret cable-wrap safety stack** — docs_biobuzz_underhand_brief_rotation_range_225, docs_biobuzz_underhand_brief_flip_rule, docs_biobuzz_turret_limelight_brief_unwrapped_angle, docs_biobuzz_turret_limelight_brief_motor_layer_enforcement, docs_biobuzz_turret_limelight_brief_stall_encoder_fault, teamcode_src_main_java_org_firstinspires_ftc_teamcode_biobuzz_turret_turret [INFERRED 0.85]
- **Cluster-center aim pipeline (tags -> center -> setpoint -> PIDF)** — docs_biobuzz_underhand_brief_tag_strip_offsets, docs_biobuzz_underhand_brief_two_tag_interpolation, docs_biobuzz_underhand_brief_cluster_center_4_tags, docs_biobuzz_underhand_brief_frame_setpoint, docs_biobuzz_underhand_brief_turret_pidf, teamcode_src_main_java_org_firstinspires_ftc_teamcode_biobuzz_vision_clusterview, teamcode_src_main_java_org_firstinspires_ftc_teamcode_biobuzz_turretaimer_turretaimer [INFERRED 0.85]
- **Gated MegaTag2 x/y pose correction flow** — docs_biobuzz_turret_limelight_brief_vision_gates, docs_biobuzz_turret_limelight_brief_latency_median_filter, docs_biobuzz_turret_limelight_brief_partial_correction, docs_biobuzz_turret_limelight_brief_megatag2_yaw_input, teamcode_src_main_java_org_firstinspires_ftc_teamcode_biobuzz_vision_posefix, teamcode_src_main_java_org_firstinspires_ftc_teamcode_biobuzz_localizer_localizer [INFERRED 0.85]

## Communities (81 total, 47 thin omitted)

### Community 0 - "SDK and Library Imports"
Cohesion: 0.07
Nodes (4): MotorMappingTest, OneMotorTesting, Constants, PoseTest

### Community 1 - "Turret Control and Tuner"
Cohesion: 0.06
Nodes (15): Driver controls on gamepad 2, Manual mode stops at ±225°, never flips, Manual turret via left_stick_x (deadband 0.05), Left-stick press toggles AUTO/MANUAL aim, Turret travel -225° to +225°, Buttons, MotorTest, Mode (+7 more)

### Community 2 - "DECODE BlueFar Auto"
Cohesion: 0.07
Nodes (27): BlueFar, PathState, AT_BAR_INTAKE_WAIT, BAR_TO_SHOOT_FOLLOW, BRAKING_1, BRAKING_2, BRAKING_BAR, BRAKING_PRELOAD (+19 more)

### Community 3 - "DECODE RedFar Auto"
Cohesion: 0.07
Nodes (27): PathState, AT_BAR_INTAKE_WAIT, BAR_TO_SHOOT_FOLLOW, BRAKING_1, BRAKING_2, BRAKING_BAR, BRAKING_PRELOAD, DONE (+19 more)

### Community 4 - "Shooter, Feeder and RPM"
Cohesion: 0.06
Nodes (17): Game rules enforced in code (G304.H, G403, G404, G407, G402, G417, G410), Is 3500 RPM enough? physics estimate, Distance->RPM tables (separate POLLEN/NECTAR), Rules that constrain the code, Distance -> motor RPM tables (POLLEN/NECTAR), Feeder, Mode, COLLECT (+9 more)

### Community 5 - "DECODE AutoTest"
Cohesion: 0.07
Nodes (26): AutoTest, PathState, AT_BAR_INTAKE_WAIT, BAR_TO_SHOOT_FOLLOW, BRAKING_1, BRAKING_2, BRAKING_3, BRAKING_BAR (+18 more)

### Community 6 - "DECODE BlueMain Auto"
Cohesion: 0.07
Nodes (26): BlueMain, PathState, AT_BAR_INTAKE_WAIT, BAR_TO_SHOOT_FOLLOW, BRAKING_1, BRAKING_2, BRAKING_3, BRAKING_BAR (+18 more)

### Community 7 - "DECODE RedMain Auto"
Cohesion: 0.07
Nodes (26): PathState, AT_BAR_INTAKE_WAIT, BAR_TO_SHOOT_FOLLOW, BRAKING_1, BRAKING_2, BRAKING_3, BRAKING_BAR, DONE (+18 more)

### Community 8 - "Localization and Vision Gating"
Cohesion: 0.08
Nodes (9): Odometry-first localization with MegaTag2 x/y corrections, Limelight<->Pedro frame helper (position and heading), Latency-compensated median offset window, Localization (odometry + Limelight), MegaTag2 yaw input = heading + turret angle, Pinpoint pod offsets disagree; measure, Vision correction gates (tags>=2, dist, staleness, speed, TIP), Localizer (+1 more)

### Community 11 - "Limelight Tag Clusters"
Cohesion: 0.10
Nodes (9): AprilTag clusters 30-45 under HIVE cells, AprilTag IDs 30-45 per CELL, HIVE rocker with two CELLS; TIP swaps up CELL, 4 tags: average for cluster center, Tag strip offsets -6.5/-2.75/+2.75/+6.5 in (tagId -> s), Tag strip order per CELL, 2-3 tags: interpolate s=0 from farthest pair, ClusterView (+1 more)

### Community 13 - "DECODE MainTeleop"
Cohesion: 0.15
Nodes (6): LedColor, GREEN, ORANGE, RED, YELLOW, MainTeleop

### Community 14 - "DECODE NewTestTeleop"
Cohesion: 0.15
Nodes (6): LedColor, GREEN, ORANGE, RED, YELLOW, NewTestTeleop

### Community 15 - "TIP Tracking and Up Cell"
Cohesion: 0.12
Nodes (7): BALLS_TO_TIP = 8 POLLEN, BALLS_TO_TIP = 8, TIP tracking (cluster ID change, shot count fallback), CellSide, AUDIENCE, FAR, TipTracker

### Community 17 - "Shot Solver and AUTO Steps"
Cohesion: 0.14
Nodes (9): canShoot gate conditions, Shoot while moving (velocity lead, v2), Shoot from anywhere shot solver, Aimed = |error| < atan(6 in / distance) and slow, DriveStep, IntakeDwellStep, ShootStep, WaitStep (+1 more)

### Community 20 - "Pedro Tuners (Drive/Heading)"
Cohesion: 0.15
Nodes (4): CentripetalTuner, DriveTuner, HeadingTuner, TranslationalTuner

### Community 21 - "Robot Hub"
Cohesion: 0.15
Nodes (6): Hardware map names, kV yaw-rate counter-rotation feedforward, Turret PIDF on encoder angle (kF static friction), TurretPIDFTuner OpMode (step, tag tracking, hold), TurretPidfTuner, Robot

### Community 22 - "DECODE 2-Player Limelight TeleOp"
Cohesion: 0.21
Nodes (5): SnapMode, NONE, PRECISE, QUICK, Teleop2PlayerLimelight

### Community 24 - "Shared Classes and Pose Handoff"
Cohesion: 0.15
Nodes (6): Turret aim via shared ShotSolver (odometry FF + tx correction), Off-robot turret sim check (run-turret-sim.sh), INIT selection menu (alliance, start, route, delay, up CELL), Lost target: hold angle or odometry aim, PoseStorage, ShotSolver

### Community 25 - "BIOBUZZ Game Rules"
Cohesion: 0.13
Nodes (17): Field AprilTags (36h11 clusters under CELLS), CELL (up-facing / down-facing), FLOWERS, G403/G404 No Powered Motion in Transition or After Match, G407 Max 4 Scoring Elements Controlled, G410 NECTAR into FLOWER Only Last 60 s, G417 Don't Meddle with the HIVE (shoot own up CELL only), GARDEN (+9 more)

### Community 27 - "DECODE Testing OpMode"
Cohesion: 0.21
Nodes (6): GainSel, KD, KF, KI, KP, Testing

### Community 28 - "Auto Hold Still"
Cohesion: 0.22
Nodes (7): G304 starting rules (still after INIT), resetPosAndIMU in INIT + LOCALIZER READY, AutoHoldStill, Start, ALLIANCE_WALL, AUDIENCE, FAR

### Community 29 - "Turret Briefs and Cable Limits"
Cohesion: 0.15
Nodes (11): Underhand robot design (ring-bearing turret, one Ø97.8 mm wheel), Turret + Limelight 3A brief (older hooded design), Limelight FOV check (±27° / ±20.5°), Brief goals (auto-aim, distance RPM, localization, shoot anywhere, AUTO), Home mark zeroing in INIT, stored in PoseStorage, TURRET_MIN/MAX ±150° soft limits (superseded), Stall / encoder sanity TURRET_FAULT, Turret-frame camera pose (116.5 mm, +37° pitch) (+3 more)

### Community 32 - "Constants and Placeholders"
Cohesion: 0.18
Nodes (11): Flywheel RPM_CAP = 3500 (hard max 6000), biobuzz_pidf.txt PIDF override on Control Hub, Shot leaves in camera direction (tx=0 = aimed), Calibration test OpModes list, Unknown values: placeholders table, Shot direction = camera direction, SHOT_YAW_OFFSET_DEG = 0, goBILDA 312 RPM turret motor (537.7 counts/rev) (+3 more)

### Community 34 - "Pedro Forward/Lateral Tuners"
Cohesion: 0.19
Nodes (3): ForwardTuner, LateralTuner, TurnTuner

### Community 37 - "Preload Routes and Red/Blue Rotation"
Cohesion: 0.20
Nodes (3): Pedro field frame + 180° rotational symmetry, AutoPreloadGarden, AutoPreloadPark

### Community 38 - "Project Guide (CLAUDE.md)"
Cohesion: 0.27
Nodes (4): biobuzz package layout (shared classes, auto/, teleop/, calibration/), BIOBUZZ 2026-27 project guide (CLAUDE.md), DECODE legacy TeamCode (reuse patterns, write new), PidfTuner

### Community 39 - "Calibration Imports and Ball Type"
Cohesion: 0.27
Nodes (3): BallType, NECTAR, POLLEN

### Community 40 - "AUTO Plan and Field Coordinates"
Cohesion: 0.20
Nodes (8): AUTO scoring (LEAVE, PARK, TIP), Baseline 30 s AUTO plan, CELL y-position conflict (18.8 vs 30.8 in), Absolute Pedro field coordinates, Key red field positions table, Must change for BIOBUZZ list, RedMain Bezier control-point bug, rotate180(pose) for blue

### Community 43 - "Alliance and Tag Ownership"
Cohesion: 0.31
Nodes (3): Alliance, BLUE, RED

### Community 49 - "Distance and DECODE Reuse"
Cohesion: 0.29
Nodes (6): Teleop chassis aim (rotation only, driver keeps translation), DECODE shooter PIDF + idle ramp + ready band, Reuse from DECODE (MainTeleop patterns), CLUSTER_HEIGHT_M = 1.14, Distance blend 0.85 Limelight / 0.15 odometry, EMA 0.35, Distance from 3D tag pose, ty fallback

### Community 50 - "360° Flip Logic"
Cohesion: 0.29
Nodes (3): Turret.chooseSetpoint(desiredYaw) algorithm, Desired robot heading = robotHeading + desiredYaw, AIM_OFFSET_DEG (cluster vs opening center)

### Community 52 - "Temporary Fixed-Turret Mode"
Cohesion: 0.33
Nodes (4): Aimer interface with ChassisAimer/TurretAimer, AUTO paths end facing the target, Fixed-mode camera pose relative to robot center on floor, TURRET_AXIS_OFFSET (default 0,0)

### Community 55 - "Underhand Camera Pose"
Cohesion: 0.40
Nodes (4): Limelight on LL Mount: +20° pitch, 0.094 m, 0.325 m lens, CAMERA_HEIGHT_M = 0.325, Camera pose: 0.094 m forward, +20° pitch, LL Mount Part 1 <4> at back of turret

### Community 57 - "Turret Tuner Modes"
Cohesion: 0.40
Nodes (5): Mode, MANUAL, OFF, STEP, TRACK

## Knowledge Gaps
- **202 isolated node(s):** `START_TO_SHOOT_FOLLOW`, `SPINUP_1`, `SHOOTING_1`, `BRAKING_1`, `SHOOT_TO_2NDSET_FOLLOW` (+197 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 314 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **47 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `BiobuzzAutoBase` connect `BIOBUZZ AUTO Engine` to `SDK and Library Imports`, `Turret Control and Tuner`, `Preload Routes and Red/Blue Rotation`, `Project Guide (CLAUDE.md)`, `AUTO Plan and Field Coordinates`, `Alliance and Tag Ownership`, `Field Geometry`, `Shot Solver and AUTO Steps`, `Temporary Fixed-Turret Mode`, `Auto Preload Flower`, `Robot Hub`, `Pedro Tuning Telemetry`, `Shared Classes and Pose Handoff`, `AUTO Route Builders`, `Auto Hold Still`, `AUTO Poses`?**
  _High betweenness centrality (0.100) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `Turret` (e.g. with `Turret rotation limits (cable wrap protection)` and `360° flip rule past ±225°`) actually correct?**
  _`Turret` has 4 INFERRED edges - model-reasoned connections that need verification._
- **What connects `START_TO_SHOOT_FOLLOW`, `SPINUP_1`, `SHOOTING_1` to the rest of the system?**
  _202 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `SDK and Library Imports` be split into smaller, more focused modules?**
  _Cohesion score 0.07058823529411765 - nodes in this community are weakly interconnected._
- **Why does `BlueFar` connect `DECODE BlueFar Auto` to `SDK and Library Imports`, `Field Geometry`, `Turret Control and Tuner`?**
  _High betweenness centrality (0.085) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `BiobuzzAutoBase` (e.g. with `One AUTO per route, red-built and rotate180() to blue` and `AUTO paths end facing the target`) actually correct?**
  _`BiobuzzAutoBase` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Should `Turret Control and Tuner` be split into smaller, more focused modules?**
  _Cohesion score 0.06015037593984962 - nodes in this community are weakly interconnected._