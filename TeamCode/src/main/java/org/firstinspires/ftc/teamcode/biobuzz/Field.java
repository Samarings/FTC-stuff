package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.geometry.Pose;

/**
 * Field geometry in the Pedro frame (inches, origin at the red-wall/audience corner,
 * +x toward the blue wall, +y toward the far wall, heading 0 = +x, CCW positive).
 *
 * The field is 180° rotationally symmetric: blue = (144 - x, 144 - y, h + π).
 * Never mirror only one axis.
 */
public final class Field {
    private Field() {}

    public static final double SIZE_IN = 144.0;
    public static final double CENTER_IN = 72.0;
    public static final double M_TO_IN = 39.3700787;

    /** Red pose -> the matching blue pose (and back). */
    public static Pose rotate180(Pose p) {
        return new Pose(SIZE_IN - p.getX(), SIZE_IN - p.getY(), wrapRad(p.getHeading() + Math.PI));
    }

    // ------------------------------------------------------------------ CELLS

    /** Field position (x, y) of one of an alliance's CELL openings. */
    public static Pose cellPosition(Alliance alliance, CellSide side) {
        if (alliance == Alliance.RED) {
            return side == CellSide.AUDIENCE
                    ? new Pose(RED_AUDIENCE_CELL_X, RED_AUDIENCE_CELL_Y, 0)
                    : new Pose(RED_FAR_CELL_X, RED_FAR_CELL_Y, 0);
        }
        // Rotating red's audience CELL by 180° lands on blue's FAR CELL, and vice versa.
        Pose redTwin = cellPosition(Alliance.RED, side.other());
        return rotate180(redTwin);
    }

    /** Height of the up CELL's opening center (aim point). */
    public static double cellAimHeightIn() {
        return 0.5 * (CELL_OPENING_BOTTOM_IN + CELL_OPENING_TOP_IN);
    }

    // ------------------------------------------------------------------ OBSTACLES / LIMITS

    /** True if (x, y) is inside the HIVE frame footprint grown by {@code inflateIn}. */
    public static boolean insideHiveFrame(double x, double y, double inflateIn) {
        double hx = HIVE_FRAME_SIZE_X_IN / 2.0 + inflateIn;
        double hy = HIVE_FRAME_SIZE_Y_IN / 2.0 + inflateIn;
        return Math.abs(x - CENTER_IN) <= hx && Math.abs(y - CENTER_IN) <= hy;
    }

    /** True if a straight segment comes within the robot's radius (+clearance) of the HIVE frame. */
    public static boolean segmentHitsHive(Pose a, Pose b) {
        double inflate = Math.max(ROBOT_HALF_LENGTH_IN, ROBOT_HALF_WIDTH_IN) + HIVE_CLEARANCE_IN;
        double len = Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
        int n = Math.max(2, (int) Math.ceil(len / 0.5));
        for (int i = 0; i <= n; i++) {
            double t = (double) i / n;
            double x = a.getX() + t * (b.getX() - a.getX());
            double y = a.getY() + t * (b.getY() - a.getY());
            if (insideHiveFrame(x, y, inflate)) return true;
        }
        return false;
    }

    /** G402 geofence: is this robot-center x legal for our alliance in AUTO? */
    public static boolean onOurHalf(Alliance alliance, double x) {
        double margin = Math.max(ROBOT_HALF_LENGTH_IN, ROBOT_HALF_WIDTH_IN);
        return alliance == Alliance.RED ? x <= CENTER_IN - margin : x >= CENTER_IN + margin;
    }

    public static boolean insideField(double x, double y) {
        return x >= 0 && x <= SIZE_IN && y >= 0 && y <= SIZE_IN;
    }

    // ------------------------------------------------------------------ ANGLES

    /** Field heading (rad) from one point toward another. */
    public static double headingToward(double fromX, double fromY, double toX, double toY) {
        return Math.atan2(toY - fromY, toX - fromX);
    }

    /** Wrap to (-π, π]. */
    public static double wrapRad(double a) {
        a = ((a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
        return a == -Math.PI ? Math.PI : a;
    }

    /** Wrap to (-180, 180]. */
    public static double wrapDeg(double a) {
        a = ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return a == -180.0 ? 180.0 : a;
    }

    // ------------------------------------------------------------------ LIMELIGHT FRAME

    /**
     * Limelight/FTC field frame (meters, origin at field center) -> Pedro (inches, corner
     * origin). Converts heading too. See RobotConstants.LL_TO_PEDRO_ROTATION_DEG.
     */
    public static Pose limelightToPedro(double xM, double yM, double yawDeg) {
        double r = Math.toRadians(LL_TO_PEDRO_ROTATION_DEG);
        double xIn = xM * M_TO_IN, yIn = yM * M_TO_IN;
        double px = xIn * Math.cos(r) - yIn * Math.sin(r) + CENTER_IN;
        double py = xIn * Math.sin(r) + yIn * Math.cos(r) + CENTER_IN;
        return new Pose(px, py, wrapRad(Math.toRadians(yawDeg) + r));
    }

    /** Pedro heading (deg) -> Limelight field-frame yaw (deg), for updateRobotOrientation(). */
    public static double pedroHeadingToLimelightYawDeg(double pedroHeadingDeg) {
        return wrapDeg(pedroHeadingDeg - LL_TO_PEDRO_ROTATION_DEG);
    }

    /** Field position of the turret yaw axis for a given robot pose. */
    public static Pose turretAxis(Pose robot) {
        double h = robot.getHeading();
        double f = TURRET_AXIS_OFFSET_FWD_IN, l = TURRET_AXIS_OFFSET_LEFT_IN;
        return new Pose(robot.getX() + f * Math.cos(h) - l * Math.sin(h),
                robot.getY() + f * Math.sin(h) + l * Math.cos(h),
                h);
    }
}
