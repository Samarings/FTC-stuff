package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.geometry.Pose;

/**
 * Hands the robot state from AUTO to TELEOP (static fields survive between OpModes while the
 * Robot Controller app stays running). TeleOp must start from this pose and must not reset
 * the Pinpoint.
 */
public final class PoseStorage {
    private PoseStorage() {}

    public static Pose pose = null;
    public static Alliance alliance = Alliance.RED;
    public static CellSide upCell = null;
    public static int tipCount = 0;
    /** System.nanoTime() when saved; TeleOp ignores very old data. */
    public static long savedAtNanos = 0;

    /** Turret encoder home, for the rotating turret later (unused while TURRET_FIXED). */
    public static Integer turretHomeTicks = null;

    public static void save(Pose p, Alliance a, CellSide up, int tips) {
        pose = p;
        alliance = a;
        upCell = up;
        tipCount = tips;
        savedAtNanos = System.nanoTime();
    }

    /** True if AUTO saved a pose within the last {@code maxAgeS} seconds. */
    public static boolean isFresh(double maxAgeS) {
        return pose != null && (System.nanoTime() - savedAtNanos) / 1e9 <= maxAgeS;
    }

    public static void clear() {
        pose = null;
        upCell = null;
        tipCount = 0;
        savedAtNanos = 0;
    }
}
