package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.ROBOT_HALF_LENGTH_IN;
import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.ROBOT_HALF_WIDTH_IN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.biobuzz.CellSide;

/**
 * AUTO poses, RED side only (blue = rotate180, done by the AUTO engine). Pedro inches.
 *
 * ALL of these are first guesses read off the manual's figures (±1-2 in) and the brief's
 * ranges. Tune them on a real field. Rules they must respect:
 *  - Start: touching a wall, on our side, not in the LOADING ZONE, not touching a FLOWER (G304).
 *  - Stay at x <= 72 - robot half-size in AUTO (G402). Avoid the HIVE frame legs.
 *  - Park: partly inside the LOADING ZONE (x 0-11, y 95-118) and NOT touching the wall (LEAVE).
 */
@Configurable
public class AutoPoses {

    public enum Start { AUDIENCE, ALLIANCE_WALL, FAR }

    // ---- start positions (robot center). Wall contact uses the robot half-size.
    public static double START_AUDIENCE_X = 40.0;      // against the audience wall, x 30-60
    public static double START_ALLIANCE_WALL_Y = 74.0; // against the red wall, y 60-88
    public static double START_FAR_X = 24.0;           // against the far wall, x 10-36

    // Headings at the start (deg). Facing the HIVE lets the Limelight see tags early.
    public static double START_AUDIENCE_H = 90.0;
    public static double START_ALLIANCE_WALL_H = 0.0;
    public static double START_FAR_H = -90.0;

    // ---- shooting spots, one per up CELL (heading is computed to face the target)
    public static double SHOOT_AUDIENCE_CELL_X = 38.0, SHOOT_AUDIENCE_CELL_Y = 20.0;
    public static double SHOOT_FAR_CELL_X = 38.0, SHOOT_FAR_CELL_Y = 124.0;

    // ---- park in the red LOADING ZONE (x 0-11, y 95-118); keep a gap to the wall
    public static double PARK_WALL_GAP_IN = 2.0;
    public static double PARK_LOWER_Y = 100.5;
    public static double PARK_UPPER_Y = 112.5;
    public static double PARK_H = 0.0;

    // ---- detour lane used to get around the HIVE frame (x of the lane)
    public static double LANE_X = 24.0;

    // ---- FLOWERS on our side (pull POLLEN from the BOTTOM opening only, G418)
    public static double FLOWER_RED_WALL_Y = 48.0;   // FLOWER 2, on the red wall
    public static double FLOWER_FAR_WALL_X = 48.0;   // FLOWER 1, on the far wall
    /** Robot center distance from the wall when the intake is at the FLOWER's bottom opening. PLACEHOLDER. */
    public static double FLOWER_PICKUP_STANDOFF_IN = 13.0;
    /** Extra distance for the approach point before creeping in. */
    public static double FLOWER_APPROACH_EXTRA_IN = 12.0;

    // ---- GARDEN sweep along the audience wall (balls at x 0-23, y 0-2)
    public static double GARDEN_SWEEP_START_X = 34.0;
    public static double GARDEN_SWEEP_END_X = 10.0;
    public static double GARDEN_SWEEP_WALL_GAP_IN = 0.5;

    // =================================================================================

    private static double hl() { return ROBOT_HALF_LENGTH_IN; }
    private static double hw() { return ROBOT_HALF_WIDTH_IN; }

    public static Pose start(Start s) {
        switch (s) {
            case AUDIENCE:
                // facing +y: back against the audience wall -> center y = half-length
                return new Pose(START_AUDIENCE_X, hl(), Math.toRadians(START_AUDIENCE_H));
            case FAR:
                return new Pose(START_FAR_X, 144.0 - hl(), Math.toRadians(START_FAR_H));
            default:
                return new Pose(hl(), START_ALLIANCE_WALL_Y, Math.toRadians(START_ALLIANCE_WALL_H));
        }
    }

    /** Shooting spot (x, y) for the given up CELL; heading 0 (the engine sets it). */
    public static Pose shootSpot(CellSide upCell) {
        return upCell == CellSide.AUDIENCE
                ? new Pose(SHOOT_AUDIENCE_CELL_X, SHOOT_AUDIENCE_CELL_Y, 0)
                : new Pose(SHOOT_FAR_CELL_X, SHOOT_FAR_CELL_Y, 0);
    }

    public static Pose park(boolean upper) {
        return new Pose(hl() + PARK_WALL_GAP_IN, upper ? PARK_UPPER_Y : PARK_LOWER_Y, Math.toRadians(PARK_H));
    }

    /** Heading (rad) that points the intake in a field direction (rad). */
    public static double headingForIntakeFacing(double fieldDirRad) {
        return fieldDirRad - Math.toRadians(org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.INTAKE_HEADING_OFFSET_DEG);
    }

    /** FLOWER 2 (red wall): approach and pickup poses, intake facing -x. */
    public static Pose[] flowerRedWall() {
        double h = headingForIntakeFacing(Math.PI);
        return new Pose[]{
                new Pose(FLOWER_PICKUP_STANDOFF_IN + FLOWER_APPROACH_EXTRA_IN, FLOWER_RED_WALL_Y, h),
                new Pose(FLOWER_PICKUP_STANDOFF_IN, FLOWER_RED_WALL_Y, h)};
    }

    /** FLOWER 1 (far wall): approach and pickup poses, intake facing +y. */
    public static Pose[] flowerFarWall() {
        double h = headingForIntakeFacing(Math.PI / 2);
        return new Pose[]{
                new Pose(FLOWER_FAR_WALL_X, 144.0 - FLOWER_PICKUP_STANDOFF_IN - FLOWER_APPROACH_EXTRA_IN, h),
                new Pose(FLOWER_FAR_WALL_X, 144.0 - FLOWER_PICKUP_STANDOFF_IN, h)};
    }

    /** GARDEN sweep start and end, intake facing -x, driving along the audience wall. */
    public static Pose[] gardenSweep() {
        double h = headingForIntakeFacing(Math.PI);
        double y = hw() + GARDEN_SWEEP_WALL_GAP_IN;
        return new Pose[]{new Pose(GARDEN_SWEEP_START_X, y, h), new Pose(GARDEN_SWEEP_END_X, y, h)};
    }
}
