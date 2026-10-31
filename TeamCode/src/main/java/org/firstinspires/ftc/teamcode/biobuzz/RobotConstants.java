package org.firstinspires.ftc.teamcode.biobuzz;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Every tunable number for the BIOBUZZ robot, in one place.
 *
 * Values marked PLACEHOLDER are unknown and must be measured. Telemetry shows a warning
 * for each placeholder that is still at its default (see {@link #placeholderWarnings()}).
 *
 * Units: field positions in Pedro inches (origin = red-wall/audience corner, +x toward the
 * blue wall, +y toward the far wall, heading 0 = +x, CCW positive). RPM is always
 * motor-encoder RPM.
 *
 * Fields are public static (not final) so Panels can edit them live.
 */
@Configurable
public class RobotConstants {

    // =====================================================================================
    // TURRET MODE
    // =====================================================================================
    /**
     * TEMPORARY: the turret is locked facing forward. The shooter + Limelight aim by turning
     * the whole robot (ChassisAimer). The turret motor is wired into the code (Turret.java)
     * but is NEVER powered while this is true. Rotating-turret aiming (TurretAimer) is not
     * written yet.
     */
    public static boolean TURRET_FIXED = true;

    // =====================================================================================
    // HARDWARE NAMES (same as DECODE)
    // =====================================================================================
    public static String INTAKE_NAME = "intake";
    public static String FEEDER_NAME = "feeder";
    public static String SHOOTER_NAME = "shooter";
    public static String LIMELIGHT_NAME = "limelight";
    /** Turret rotation motor. Optional while TURRET_FIXED (missing hardware is fine). */
    public static String TURRET_NAME = "turret";

    // =====================================================================================
    // TURRET ROTATION (wired but locked while TURRET_FIXED). Cable-wrap safety: no slip ring.
    // =====================================================================================
    /** Flip if turning the turret counter-clockwise (seen from above) makes the angle go DOWN. */
    public static boolean TURRET_REVERSED = false;
    /**
     * Encoder ticks per turret degree. PLACEHOLDER 0 = NOT CALIBRATED: the turret is never
     * powered (limits can't be enforced without it). Measure with TurretCalibration.
     */
    public static double TURRET_TICKS_PER_DEG = 0.0;
    /** Soft limits from home, unwrapped degrees. PLACEHOLDER ±150 until measured; total < 360. */
    public static double TURRET_MIN_DEG = -150.0;
    public static double TURRET_MAX_DEG = 150.0;
    /** Power tapers linearly to 0 over this distance before a limit. */
    public static double TURRET_SLOW_ZONE_DEG = 20.0;
    /** Setpoints are clamped this far inside the limits. */
    public static double TURRET_LIMIT_MARGIN_DEG = 5.0;
    /** Angle outside [MIN - this, MAX + this] = encoder fault (stop until re-zeroed). */
    public static double TURRET_FAULT_BEYOND_DEG = 20.0;
    public static double TURRET_MAX_POWER = 0.5;
    public static double TURRET_KP = 0.02;   // power per degree, PLACEHOLDER (tune when enabled)
    public static double TURRET_KD = 0.0005;
    public static double TURRET_DEADBAND_DEG = 0.5;
    /** Stall: |power| above this but the turret moved less than STALL_MIN_DEG for STALL_TIME_S. */
    public static double TURRET_STALL_POWER = 0.3;
    public static double TURRET_STALL_MIN_DEG = 1.0;
    public static double TURRET_STALL_TIME_S = 0.3;
    /** Current fault (amps); 0 = disabled. PLACEHOLDER: set after measuring normal current. */
    public static double TURRET_STALL_CURRENT_A = 0.0;
    /** Faster than this between two loops = impossible encoder jump (fault). PLACEHOLDER. */
    public static double TURRET_MAX_PLAUSIBLE_DEG_S = 1000.0;
    /** Margin subtracted from the hand-measured cable limits when suggesting soft limits. */
    public static double TURRET_CAL_SAFETY_MARGIN_DEG = 15.0;

    // =====================================================================================
    // ROBOT SIZE (PLACEHOLDER: 18 in starting cube; measure the real frame)
    // =====================================================================================
    public static double ROBOT_HALF_LENGTH_IN = 9.0;
    public static double ROBOT_HALF_WIDTH_IN = 9.0;

    // =====================================================================================
    // FLYWHEEL (one goBILDA 5203 on the turret)
    // =====================================================================================
    public static double SHOOTER_TICKS_PER_REV = 28.0;
    /** Motor's physical top speed. RPM_CAP may never exceed this. */
    public static final double RPM_HARD_MAX = 6000.0;
    /** Normal operating cap. Raise (never above 6000) only if far shots fall short. */
    public static double RPM_CAP = 3500.0;
    public static boolean SHOOTER_REVERSED = false;

    // Custom PIDF in power units (from DECODE MainTeleop). Re-tune kF for the new wheels/hood.
    public static double SHOOTER_KP = 0.015;
    public static double SHOOTER_KI = 0.0;
    public static double SHOOTER_KD = 0.0001;
    public static double SHOOTER_KF = 0.000422143;
    public static double SHOOTER_INTEGRAL_CLAMP = 5000.0;

    /** Open-loop idle (TeleOp only), ramped to avoid brownouts. */
    public static double IDLE_RPM = 1200.0;
    public static double SHOOTER_RAMP_PER_SEC = 3.0;

    /** Kick at full power for this long when spinning up from stopped. */
    public static double SHOOTER_KICK_SEC = 0.15;

    /** "Ready" band: measured RPM within this many RPM of the target... */
    public static double SHOOTER_READY_TOL_RPM = 120.0;
    /** ...for this many consecutive loops. */
    public static int SHOOTER_READY_LOOPS = 4;

    /** Shooter cuts off if it's at >= 60% power but reads < 150 RPM for this long. */
    public static double SHOOTER_FAULT_TIME_S = 1.0;

    /** Shot detection: a drop this far below target while feeding counts as one launched ball. */
    public static double SHOT_DIP_RPM = 250.0;
    /** The detector re-arms once RPM is back within this band. */
    public static double SHOT_REARM_RPM = 120.0;

    // RPM tables: distance (in, turret axis to CELL opening, horizontal) -> motor RPM.
    // PLACEHOLDER: straight line ~2000 RPM at 1 ft to 3500 at max range. Fill these from
    // ShooterTuning. Distances must increase.
    public static double[] POLLEN_DIST_IN = {12, 36, 60, 84, 108, 132};
    public static double[] POLLEN_RPM = {2000, 2250, 2500, 2750, 3000, 3250};
    public static double[] NECTAR_DIST_IN = {12, 36, 60, 84, 108, 132};
    public static double[] NECTAR_RPM = {2000, 2250, 2500, 2750, 3000, 3250};

    // =====================================================================================
    // INTAKE / FEEDER (no servos: the feeder motor gates balls into the flywheel)
    // =====================================================================================
    public static boolean INTAKE_REVERSED = true;   // DECODE: REVERSE
    public static boolean FEEDER_REVERSED = false;  // DECODE: FORWARD

    public static double INTAKE_COLLECT_POWER = 1.0;
    /**
     * Feeder power while intaking. No stopper servos: the feeder motor is the only thing
     * keeping balls out of the flywheel, so keep this 0 (or slightly negative to hold balls
     * back) unless the mechanism can stage balls without reaching the flywheel.
     */
    public static double FEEDER_COLLECT_POWER = 0.0;
    public static double INTAKE_EJECT_POWER = 1.0;
    public static double FEEDER_EJECT_POWER = 1.0;

    // Pulsed feed while shooting (DECODE AUTO values)
    public static double FEED_INTAKE_POWER = 1.0;
    public static double FEED_FEEDER_POWER = 1.0;
    public static double FEED_PULSE_ON_SEC = 0.11;
    public static double FEED_PULSE_OFF_SEC = 0.08;


    /** G407: never control more than this many scoring elements. */
    public static int MAX_BALLS = 4;
    /** Balls held at the start of AUTO (G304: exactly 4 pre-loads). */
    public static int PRELOAD_BALLS = 4;

    /**
     * Which way the intake faces, relative to robot forward (the shooter/camera side).
     * 0 = intake at the front, 180 = intake at the back. PLACEHOLDER: confirm on the robot.
     */
    public static double INTAKE_HEADING_OFFSET_DEG = 0.0;


    // =====================================================================================
    // CAMERA (Limelight 3A, on the turret, fixed facing forward for now)
    // =====================================================================================
    /** Lens height above the tiles. PLACEHOLDER estimate; measure on the built robot. */
    public static double CAMERA_HEIGHT_M = 0.354;
    public static double CAMERA_PITCH_DEG = 37.0;
    /** Lens forward of the turret yaw axis (CAD). */
    public static double CAMERA_FORWARD_OF_TURRET_M = 0.1165;

    /**
     * Turret yaw axis relative to the robot rotation center, inches (forward, left).
     * PLACEHOLDER (0, 0): tape-measure it.
     */
    public static double TURRET_AXIS_OFFSET_FWD_IN = 0.0;
    public static double TURRET_AXIS_OFFSET_LEFT_IN = 0.0;

    public static int APRILTAG_PIPELINE = 0;

    /**
     * Limelight field frame -> Pedro frame. Pedro = rotate(LL meters * 39.37, this angle)
     * + (72, 72); Pedro heading = LL yaw + this angle. 0 means the BIOBUZZ field map was
     * built with origin at field center, +x toward the blue wall, +y toward the far wall.
     * VERIFY with LocalizationTest (park facing +x at a known spot).
     */
    public static double LL_TO_PEDRO_ROTATION_DEG = 0.0;

    // =====================================================================================
    // FIELD (red-side values; blue = rotate180). Confirm everything from field CAD.
    // =====================================================================================
    public static double RED_HIVE_PIVOT_X = 59.3;
    // ⚠ These conflict with the manual's ~18.8 in CELL spacing (would give 62.6 / 81.4).
    public static double RED_AUDIENCE_CELL_X = 59.3;
    public static double RED_AUDIENCE_CELL_Y = 56.6;
    public static double RED_FAR_CELL_X = 59.3;
    public static double RED_FAR_CELL_Y = 87.4;

    /** Up CELL opening: bottom lip ~54 in (team), top ~65.6 in (manual). */
    public static double CELL_OPENING_BOTTOM_IN = 54.0;
    public static double CELL_OPENING_TOP_IN = 65.6;
    /** Half-width used for the aim tolerance (opening is ~20 in wide; keep a margin). */
    public static double CELL_AIM_HALF_WIDTH_IN = 6.0;
    /** Height of the downward-facing tag cluster. PLACEHOLDER estimate (brief: ~45 in). */
    public static double TAG_CLUSTER_HEIGHT_IN = 45.0;

    /** HIVE frame footprint (obstacle), centered on (72, 72). */
    public static double HIVE_FRAME_SIZE_X_IN = 49.5;
    public static double HIVE_FRAME_SIZE_Y_IN = 39.0;
    /** Extra clearance around the HIVE frame for path checks. */
    public static double HIVE_CLEARANCE_IN = 2.0;

    // =====================================================================================
    // AIMING (chassis aim while TURRET_FIXED)
    // =====================================================================================
    public static double AIM_KP = 0.025;          // turn power per degree (retune: new robot)
    public static double AIM_KD = 0.0015;
    public static double AIM_MAX_POWER = 0.7;
    public static double AIM_FINE_MAX_POWER = 0.30;
    public static double AIM_FINE_ZONE_DEG = 4.0;
    public static double AIM_DEADBAND_DEG = 0.5;
    /** Constant yaw offset from cluster center to opening center when aiming on tx. */
    public static double AIM_TX_OFFSET_DEG = 0.0;
    /** EMA filter on tx (DECODE 0.35). */
    public static double TX_EMA_ALPHA = 0.35;
    /** Max robot turn rate allowed while launching. */
    public static double SHOOT_MAX_TURN_RATE_DEG_S = 30.0;
    /** Max robot speed while launching (stationary shots first; raise with shoot-while-moving). */
    public static double SHOOT_MAX_SPEED_IN_S = 10.0;
    /** Don't shoot closer than this (target leaves the camera frame; steep shot). PLACEHOLDER. */
    public static double MIN_SHOT_DIST_IN = 24.0;
    /** Optional: fuse Limelight-measured distance into the solver (0 = odometry only). */
    public static double VISION_DISTANCE_WEIGHT = 0.0;

    // Shoot-while-moving (v2). Off until stationary shots work.
    public static boolean SHOOT_WHILE_MOVING = false;
    public static double TIME_OF_FLIGHT_S = 0.6; // PLACEHOLDER

    // =====================================================================================
    // TIP TRACKING
    // =====================================================================================
    public static int BALLS_TO_TIP = 8;
    /** Pause shooting this long after a TIP while the HIVE swings. */
    public static double TIP_SETTLE_S = 1.25;
    /** Frames the new cluster must be seen before a TIP is confirmed. */
    public static int TIP_CONFIRM_FRAMES = 4;

    // =====================================================================================
    // LOCALIZATION: Limelight MegaTag2 corrections (x/y only)
    // =====================================================================================
    public static boolean VISION_CORRECTIONS_ENABLED = true;
    public static int VISION_MIN_TAGS = 2;
    public static double VISION_MAX_AVG_DIST_M = 2.2;
    public static double VISION_MAX_STALENESS_MS = 100.0;
    public static double VISION_MAX_SPEED_IN_S = 15.0;
    public static double VISION_MAX_TURN_RATE_DEG_S = 45.0;
    /** Reject vision within this long of any HIVE motion (ours or the opponent's). */
    public static double VISION_TIP_BLACKOUT_S = 2.0;
    public static double ODOM_HISTORY_S = 0.5;
    public static int VISION_WINDOW_MIN_FRAMES = 5;
    public static int VISION_WINDOW_MAX_FRAMES = 10;
    public static double VISION_WINDOW_MAX_AGE_S = 1.0;
    public static double VISION_CORRECTION_PERIOD_S = 2.5;
    public static double VISION_MIN_OFFSET_IN = 0.5;
    public static double VISION_PARTIAL_MAX_IN = 12.0;
    public static double VISION_ALPHA = 0.6;
    public static int VISION_BIG_AGREE_WINDOWS = 3;
    public static double VISION_BIG_AGREE_IN = 2.0;

    // =====================================================================================
    // AUTO
    // =====================================================================================
    /** Start the drive to the LOADING ZONE no later than this (measure drive times!). */
    public static double AUTO_PARK_DEADLINE_S = 25.0;
    /** Stop shooter/intake/feeder at this time. */
    public static double AUTO_MECH_STOP_S = 29.5;
    public static double AUTO_SHOOT_STEP_TIMEOUT_S = 4.5;
    public static double AUTO_INTAKE_DWELL_S = 1.2;
    public static double AUTO_DRIVE_STEP_TIMEOUT_S = 5.0;
    /** Slow speed for creeping into POLLEN sources. */
    public static double AUTO_INTAKE_DRIVE_POWER = 0.45;
    /** Re-issue the hold-point when the aim heading moves more than this. */
    public static double AUTO_REHOLD_DEG = 0.75;
    /** Rate limit for Limelight tx fine-correction of the hold heading (deg per second). */
    public static double AUTO_TX_CORRECT_DEG_S = 20.0;

    // =====================================================================================
    // TELEOP
    // =====================================================================================
    public static double TELEOP_SLOW_SCALE = 0.4;
    public static double TELEOP_TURN_SCALE = 0.8;
    public static double TELEOP_RPM_TRIM_STEP = 50.0;
    public static double TELEOP_LENGTH_S = 120.0;

    // =====================================================================================

    /** Hard-clamped flywheel cap: RPM_CAP, but never above the motor's 6000. */
    public static double rpmCap() {
        return Math.max(0.0, Math.min(RPM_CAP, RPM_HARD_MAX));
    }

    /** One line per placeholder still at its default, for telemetry. */
    public static java.util.List<String> placeholderWarnings() {
        java.util.List<String> w = new java.util.ArrayList<>();
        if (TURRET_AXIS_OFFSET_FWD_IN == 0.0 && TURRET_AXIS_OFFSET_LEFT_IN == 0.0)
            w.add("TURRET_AXIS_OFFSET is (0,0): tape-measure it");
        if (CAMERA_HEIGHT_M == 0.354) w.add("CAMERA_HEIGHT_M is the 0.354 m estimate");
        if (TAG_CLUSTER_HEIGHT_IN == 45.0) w.add("TAG_CLUSTER_HEIGHT_IN is the 45 in estimate");
        if (RED_AUDIENCE_CELL_Y == 56.6 && RED_FAR_CELL_Y == 87.4)
            w.add("CELL positions unverified (conflict with 18.8 in spacing)");
        if (POLLEN_RPM[0] == 2000 && POLLEN_RPM[POLLEN_RPM.length - 1] == 3250)
            w.add("POLLEN RPM table is the placeholder line");
        if (ROBOT_HALF_LENGTH_IN == 9.0 && ROBOT_HALF_WIDTH_IN == 9.0)
            w.add("Robot size is the 18 in cube placeholder");
        if (org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants.forwardPodY == 0.0
                && org.firstinspires.ftc.teamcode.pedroPathing.Constants.localizerConstants.strafePodX == 0.0)
            w.add("Pinpoint pod offsets are (0,0): measure them (pedroPathing/Constants.java)");
        if (!TURRET_FIXED && TURRET_TICKS_PER_DEG == 0.0)
            w.add("TURRET_TICKS_PER_DEG not set: turret stays unpowered");
        if (!TURRET_FIXED && TURRET_MIN_DEG == -150.0 && TURRET_MAX_DEG == 150.0)
            w.add("Turret limits are the ±150° placeholder: measure them");
        return w;
    }
}
