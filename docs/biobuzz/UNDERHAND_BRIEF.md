# Underhand turret brief (current design, FTC BIOBUZZ 2026-27)

**This is the current robot design.** It replaces the hooded "Turret maybe" design. This
document is a brief for writing the code: it says what to build, and the code is written
separately.

Source: Onshape document **"underhand maybe" → Assembly 1** (74 instances, 15 mates, all
Fastened), measured Nov 11 2026, plus the team's turret specs.

## What carries over from `TURRET_LIMELIGHT_BRIEF.md`

These sections still apply unchanged. Read them there:
- **Field coordinates:** Pedro frame, `rotate180()` for blue, key field positions.
- **Localization:** Pinpoint odometry is the main source, with Limelight x/y corrections
  every few seconds.
- **Shot solver concept** (aim + RPM + `canShoot`), **TIP tracking**, and
  `BALLS_TO_TIP = 8`.
- **Autonomous (AUTO)** plan, INIT menu, start positions and AUTO rules.
- **Rules that constrain the code**, and **Reuse from DECODE**.
- **RPM:** cap 3500 by default (never above 6000), with all RPM numbers editable.
- Turret safety mechanics: unwrapped angle, home mark, motor-layer limit enforcement,
  stall/encoder fault cut-off, and `PoseStorage` (no re-zero in teleop).

**Superseded by this document:**
- The old CAD hardware section and camera pose (37° pitch, 116.5 mm offset).
- The ±150° / "< 360° total" turret limits.
- The "TEMPORARY fixed turret" mode. **The turret rotates now.**

## CAD findings ("underhand maybe")

CAD frame: Z is up, and the turret bearing lies flat. Values are mm unless noted.

**The assembly is mid-build.** The shooter wheel (Core + Tire), its 8 mm shaft
(2106-4008-1200), the hub (1310-0016-4008), two bearing blocks and the POLLEN model
float above the turret, not mounted. So the wheel position and launch geometry **can't
be read from CAD yet**.

| Item | What CAD shows |
|---|---|
| Turret bearing | Same ring bearing as before: outer race Ø144, inner race Ø124. **Yaw axis** is vertical through CAD (X −23.0, Y 5.0). |
| Turret height | Side-plate tops are 119 mm (4.7 in) above the bearing's bottom face. |
| Side plates | "Side Plate Motor New" at X 27–33 and "Side Plate No Motor New" at X −79…−73. 6 mm thick, 100 mm tall, spaced 100 mm apart inside. Both are centered on the yaw axis (X −23). |
| Shooter motor | One goBILDA **5203** Yellow Jacket (ratio not in CAD) on the side plates. Its axis runs along X at (Y −77.9, Z 65.7), on the **front (−Y)** side. |
| Shooter wheel (unplaced) | Tire Ø97.8 mm (≈ 3.85 in), 24 mm wide, on a 90 mm core. One wheel. |
| Ramp ("New Ramp") | Between the plates at the back: Y 41–86, Z 28–80. The top surface is tilted ~25° and slopes **down toward +Y**. |
| Other parts | "Part Studio 2" plate at the front (Y −78…−44). Two goBILDA 1205 dual block mounts. A base plate under the bearing. |
| **Limelight mount** | **Part 1 <4> ("LL Mount")** at the **back of the turret (+Y)**. Details below. |

### Camera pose (from Part 1 <4>, the LL Mount)

- **Limelight 3A position.** The Limelight itself isn't modeled. Its 4-hole pattern
  (64 × 40 mm) on the mount is centered at CAD (X −23.0, Y 99.4, Z 50.0).
- **On the turret centerline.** The hole-pattern center is at the yaw axis X.
- **Facing.** The mount plate's face normal is (0, 0.940, 0.342). The camera faces
  **CAD +Y** (the back of the turret) and is **pitched up 20°**. (The plate is 20° off
  vertical.)
- **Offsets in the turret frame** (origin on the yaw axis at the bearing's bottom face,
  +forward = camera direction):

  | Measurement | Value |
  |---|---|
  | Forward from yaw axis | ≈ **0.094 m** to the hole-pattern center. The lens sits ~1–2 cm further out along the camera axis. **Measure it.** |
  | Left/right | **0** |
  | Up from the bearing's bottom face | ≈ **0.033 m** to the hole-pattern center (lens a few mm higher) |
  | Pitch | **+20°**; yaw 0, roll 0 |

### Heights (team: "same height still applies", robot + turret ≈ 16 in)

| Point | Estimate |
|---|---|
| Bearing bottom face | 16 in − 4.7 in ≈ **11.3 in (0.287 m)** above the tiles |
| Limelight lens | ≈ 0.287 + 0.033 + a few mm ≈ **0.325 m (12.8 in)** |

Use `CAMERA_HEIGHT_M = 0.325` and replace it with a tape-measure value.

### What the 20° camera can see

- Vertical FOV is ±20.5°, so the camera sees about −0.5° to 40.5° above horizontal.
- The tag cluster is ≈ 1.14 m (≈ 45 in) up; measure it on the field. That leaves a
  height difference of ≈ 0.82 m.
- So **within about 0.95 m (37 in) horizontal of the cluster, the tags leave the top of
  the frame.**
- Close shots must aim from odometry (shot solver fallback). There is no upper FOV
  limit; far detection is limited by how oblique the tags look.

### Shot direction (confirmed by the team)

**The ball leaves the turret in the same direction the Limelight faces** (CAD +Y).
- `SHOT_YAW_OFFSET_DEG = 0`, and "tx = 0 on the cluster center" means "aimed".
- Turret forward (0°) = the camera and shot direction.
- The shooter motor sitting on the −Y side doesn't change this.

## Turret hardware (team specs)

- **Motor:** goBILDA **312 RPM** Yellow Jacket (5203 series, 19.2:1). Encoder:
  **537.7 counts per output-shaft revolution** (28 counts/motor rev × 19.2).
- **Turret:** goBILDA turret, driven by that motor. **The turret gear ratio is not
  known yet**, so it's `TURRET_GEAR_RATIO` = motor output revolutions per one full
  turret revolution.
- `TURRET_TICKS_PER_DEG = 537.7 × TURRET_GEAR_RATIO / 360`. Confirm by measurement in
  `TurretCalibration`: turn the turret by hand 90° and read the ticks. **Turret
  auto-aim stays disabled until this is set.**

## Turret rotation range and the 360° flip (team spec)

- **0° = start position** (home mark). Set by hand before INIT, recorded in INIT with
  no movement (G304.H). Carried to teleop via `PoseStorage`.
- **Travel: −225° to +225°** (CW and CCW). This replaces the old ±150° default.
  - The turret angle stays **unwrapped** (never wrapped to ±180).
  - Use `TURRET_MIN_DEG = −225` and `TURRET_MAX_DEG = +225` as constants.
  - **Check the wiring:** with all cables connected, turn the turret by hand to ±225°
    and confirm nothing pulls tight. The Limelight USB and the shooter motor's cables
    cross the joint.
- **Flip rule:** while tracking, if the target direction would require going past
  +225°, command the same direction **−360°** (e.g. +230° → −130°). Past −225°,
  command **+360°**. The turret spins about 360° the other way and keeps tracking.
  - Because ±225° overlaps by 90° (135°…225° and −225°…−135°), a flip always lands
    well inside the range. Add a few degrees of hysteresis (flip at |angle| > 225° + 3°
    requested) so it doesn't chatter at the boundary.
  - **During a flip: `canShoot = false`**, the feeder stays off, and telemetry shows
    "FLIPPING". The camera loses the target during the spin, so drive the flip from
    the encoder setpoint, then resume tracking.
  - Outside a flip, always pick the legal angle closest to the current angle.
- **Manual mode** stops at ±225°; it never flips on its own.
- The old brief's **motor-layer enforcement** still applies with these limits, in every
  mode: zero power past a limit, slow-down zone near it, stall/encoder fault cut-off.

## Driver controls (team spec)

The team wrote "left stick". The gamepad isn't specified. **Assume gamepad 2
(operator)**, since gamepad 1's left stick drives the robot. Confirm with the team.

- **Left stick press (toggle): AUTO aim ↔ MANUAL.** Edge-detected so one press = one
  toggle. Show the mode on telemetry and the LEDs.
  - Conflict: DECODE used G2 left-stick press for auto-RPM, so move that.
- **MANUAL:** left stick left/right (`left_stick_x`) turns the turret.
  - Deadband ~0.05; scale to a max manual speed.
  - Still limited to ±225°, with no flip.
  - Pick a sign convention (stick right = turret clockwise seen from above) and
    document it.
- **AUTO:** Limelight + PIDF tracking (below). If the Limelight loses the target, fall
  back to odometry aim. With no pose either, hold the current angle.

## AUTO aim with the Limelight (team spec)

**Target:** our alliance's **up-facing CELL**, marked by its 4-tag cluster on the
underside of the HIVE (manual §9.6/9.9).

The team says "aim at the center of the hive". The cluster is centered on its CELL,
so **aim at the cluster center = the center of our up CELL**. Keep a tunable
`AIM_OFFSET_DEG` for the small difference between the cluster center and the opening
center.

**Tag IDs and order** (manual Fig. 9-17 top view, left→right as drawn). **Verify the
order on a real field with `VisionCheck`.**

| CELL | IDs in strip order |
|---|---|
| Red far side | 33, 32, 31, 30 |
| Red audience side | 34, 35, 36, 37 |
| Blue far side | 45, 44, 43, 42 |
| Blue audience side | 38, 39, 40, 41 |

Each tag's position along the strip, from the cluster center (manual Fig. 9-15):
**−6.5, −2.75, +2.75, +6.5 in** for the 1st, 2nd, 3rd and 4th tag in strip order.
Store this as a map `tagId → s` (inches).

**Finding the center** (team spec: use all 4, else 2). Only use our alliance's IDs:
- **4 tags seen:** cluster center = average of the 4 tag positions. The layout is
  symmetric, so averaging tx works, or average the 3D tag poses.
- **2 or 3 tags seen:** use two of them, preferably the pair farthest apart, with their
  known `s` values to find where `s = 0` lies:
  `tx_center = tx_A + (0 − s_A) × (tx_B − tx_A) / (s_B − s_A)`.
  - This also works when both tags are on one side of the center (extrapolation).
  - Do the same with ty if needed.
  - With 3 tags, a least-squares line through all 3 is fine, but optional.
- **1 tag (not in team spec, suggestion):** don't aim from it alone. Fall back to
  odometry aim.
- **0 tags:** odometry aim (shot solver).
- Ignore the opponent's clusters and any frame from the CELL we believe is down. Pause
  ~1–1.5 s after a TIP (old brief: "TIP tracking").

### PIDF control (team spec) + tuner program

- **Don't PID directly on tx.** The camera updates slowly and with lag. Instead, each
  new Limelight frame sets a **turret angle setpoint** =
  `turretAngleAtFrameTime + tx_center + AIM_OFFSET_DEG`, passed through the
  ±225°/flip rule. Look up the turret angle at the frame time from a short history,
  using the frame's latency.
- **PIDF on the turret encoder angle** every loop:
  `power = kP·e + kI·∫e + kD·de/dt + kF·sign(e)`.
  - kF is static friction, so the turret moves for small errors.
  - Optional: feedforward `kV × (−robotYawRate)`, so the turret counter-rotates while
    the chassis turns.
  - Clamp the integral, use a small error deadband, and clamp the output to max power.
- **`TurretPIDFTuner` OpMode:**
  - kP, kI, kD, kF, kV, deadband and max power editable live via **Panels**
    `@Configurable` (Panels is already a project dependency).
  - **Modes:**
    - **step test:** jumps between two angles, e.g. 0° ↔ 90°, and 0° ↔ −200° to
      exercise the flip.
    - **tag tracking:** real Limelight.
    - **manual hold.**
  - Graph setpoint, angle and error on Panels. Show overshoot and settle time on
    telemetry.
  - Final gains are copied into the constants file by hand.
- **Aimed** (for `canShoot`) = |turret error| < `atan(6 in / distance)` and turret speed
  low, for a few loops.

### Distance from the Limelight → RPM (team spec)

- **Distance** = horizontal distance from the turret to the cluster center.
  - **Preferred:** the 3D pose of the visible tags (`getTargetPoseCameraSpace` per
    fiducial; verify the method names on the SDK in use), averaged and converted to
    horizontal distance.
  - **Fallback:** `d = (CLUSTER_HEIGHT_M − CAMERA_HEIGHT_M) / tan(20° + ty_center)`.
  - Filter it (the DECODE code's EMA, α ≈ 0.35, is fine).
  - When tags aren't visible, use the odometry distance from the shot solver. Blend
    them like DECODE did (~0.85 Limelight / 0.15 odometry when both exist).
- **RPM** = interpolated `distance → motor RPM` table, separate for POLLEN and NECTAR.
  - Clamp to `RPM_CAP` (3500 default, never above 6000).
  - Built with `ShooterTuning`.
  - RPM is measured at the motor encoder, so the shooter gear ratio isn't needed.

## Unknowns: placeholders, don't block on them

| Value | Placeholder | How to get it |
|---|---|---|
| `TURRET_GEAR_RATIO` / `TURRET_TICKS_PER_DEG` | unset → auto-aim off | goBILDA turret spec, then `TurretCalibration` |
| Turret direction sign | +power = CCW from above = +deg | `TurretCalibration` |
| Camera forward offset / height | 0.094 m / 0.325 m | Tape measure to the lens |
| `CLUSTER_HEIGHT_M` | 1.14 | Field CAD or a practice field |
| Tag strip order | the table above | `VisionCheck` on a real cluster |
| `TURRET_AXIS_OFFSET` (yaw axis vs robot center) | (0, 0) | Tape measure. It matters for odometry aim and localization (see old brief). |
| Gamepad for turret controls | gamepad 2 | Ask the team |
| PIDF gains | small kP only, everything else 0 | `TurretPIDFTuner` |
| RPM tables | placeholder line ≤ 3500 | `ShooterTuning` |

## Test OpModes to write (TeleOp group "Calibration")

- `TurretCalibration`: ticks/deg (turn by hand 90°), direction, ±225° wiring check,
  home mark.
- `TurretPIDFTuner`: see above.
- `VisionCheck`: visible IDs, strip order, tag count, cluster center tx/ty, which
  fallback is active (4 / 2–3 / 1 / 0 tags), and distance.
- `ShooterTuning`: distance → RPM points for POLLEN and NECTAR.
