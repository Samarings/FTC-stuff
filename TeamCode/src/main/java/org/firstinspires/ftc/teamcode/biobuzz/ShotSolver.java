package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.geometry.Pose;

/**
 * "Shoot from anywhere": one solver shared by AUTO and TeleOp, run every loop.
 *
 * 1. Target = our up CELL's opening (TipTracker says which CELL).
 * 2. Odometry feed-forward: desired direction from the turret axis to the target.
 * 3. Vision fine-tune: when our up CELL's cluster is visible, aim on its (filtered) tx.
 * 4. Distance (turret axis -> opening, horizontal) -> RPM from the POLLEN or NECTAR table.
 * 5. canShoot gate. The feeder only runs while it's true: this is how we satisfy G417.
 *
 * TURRET_FIXED: the turret angle is 0, so the aim error is a robot-heading error and the
 * output is a desired robot heading. The target is always "reachable" (no turret dead zone).
 */
public class ShotSolver {

    public static class Solution {
        public CellSide upCell;
        public Pose target;                 // opening center (x, y)
        public double distanceIn;
        public double desiredHeadingRad;    // field heading that points the shooter at the target
        public double odomAimErrDeg;        // desired - current, from odometry
        public boolean visionAim;           // aim error came from Limelight tx
        public double aimErrDeg;            // the error the Aimer should drive to 0 (+ = turn CCW)
        public double toleranceDeg;
        public double rpm;
        public boolean aimed;
        public boolean canShoot;
        public String whyNot = "";
    }

    private final Alliance alliance;
    private double txFilt = 0.0;
    private boolean txInit = false;

    public ShotSolver(Alliance alliance) {
        this.alliance = alliance;
    }

    /**
     * @param ballsHeld          estimated balls on board, or -1 if unknown (TeleOp, no sensor)
     * @param periodAllowsShots  false during the AUTO->TELEOP transition, after the match, etc.
     * @param rpmTrim            driver trim added to the table RPM (0 in AUTO)
     */
    public Solution solve(Localizer loc, Vision vision, TipTracker tips, Shooter shooter,
                          BallType ball, int ballsHeld, boolean periodAllowsShots,
                          double rpmTrim, double nowSec) {
        Solution s = new Solution();
        Pose robot = loc.getPose();
        s.upCell = tips.getUpCell();
        Pose cell = Field.cellPosition(alliance, s.upCell);
        s.target = cell;

        Pose axis = Field.turretAxis(robot);
        double aimX = cell.getX(), aimY = cell.getY();
        double distNow = Math.hypot(aimX - axis.getX(), aimY - axis.getY());
        if (SHOOT_WHILE_MOVING) {
            // v2: lead the target by our own velocity over the time of flight
            aimX -= loc.getVelX() * TIME_OF_FLIGHT_S;
            aimY -= loc.getVelY() * TIME_OF_FLIGHT_S;
        }

        s.desiredHeadingRad = Field.headingToward(axis.getX(), axis.getY(), aimX, aimY);
        s.odomAimErrDeg = Math.toDegrees(Field.wrapRad(s.desiredHeadingRad - robot.getHeading()));

        // ---- vision fine-tune (fast, per-frame): only our up CELL's cluster
        Vision.ClusterView cv = vision.ourCluster(s.upCell);
        if (cv.visible()) {
            txFilt = txInit ? TX_EMA_ALPHA * cv.txDeg + (1 - TX_EMA_ALPHA) * txFilt : cv.txDeg;
            txInit = true;
            // tx > 0 = target right of center = turn clockwise = negative error
            s.aimErrDeg = -(txFilt - AIM_TX_OFFSET_DEG);
            s.visionAim = true;
        } else {
            txInit = false;
            s.aimErrDeg = s.odomAimErrDeg;
            s.visionAim = false;
        }

        // ---- distance + RPM
        double dist = distNow;
        if (VISION_DISTANCE_WEIGHT > 0 && cv.visible() && !Double.isNaN(cv.distanceIn)) {
            dist = VISION_DISTANCE_WEIGHT * cv.distanceIn + (1 - VISION_DISTANCE_WEIGHT) * distNow;
        }
        s.distanceIn = dist;
        double[] xs = ball == BallType.NECTAR ? NECTAR_DIST_IN : POLLEN_DIST_IN;
        double[] ys = ball == BallType.NECTAR ? NECTAR_RPM : POLLEN_RPM;
        s.rpm = Shooter.clampRpm(interpolate(xs, ys, dist) + rpmTrim);

        // ---- aim tolerance: half the usable opening width at this range
        s.toleranceDeg = Math.toDegrees(Math.atan2(CELL_AIM_HALF_WIDTH_IN, Math.max(1.0, dist)));
        s.aimed = Math.abs(s.aimErrDeg) <= s.toleranceDeg;

        // ---- canShoot (first failing reason is reported)
        String why = null;
        if (Double.isNaN(robot.getX()) || Double.isNaN(robot.getY())) why = "POSE NaN";
        else if (!periodAllowsShots) why = "MATCH PERIOD";
        else if (tips.inSettleWindow(nowSec)) why = "TIP SETTLING";
        else if (ballsHeld == 0) why = "NO BALLS";
        else if (dist < MIN_SHOT_DIST_IN) why = "TOO CLOSE";
        else if (Field.insideHiveFrame(robot.getX(), robot.getY(), 0)) why = "UNDER HIVE";
        else if (!s.aimed) why = String.format("AIM %.1f° > %.1f°", Math.abs(s.aimErrDeg), s.toleranceDeg);
        else if (loc.getTurnRateDeg() > SHOOT_MAX_TURN_RATE_DEG_S) why = "TURNING";
        else if (!SHOOT_WHILE_MOVING && loc.getSpeed() > SHOOT_MAX_SPEED_IN_S) why = "MOVING";
        else if (shooter.getFault() != null) why = "SHOOTER FAULT: " + shooter.getFault();
        else if (!shooter.isReady()) why = "RPM";
        else if (Math.abs(shooter.getTargetRpm() - s.rpm) > SHOOTER_READY_TOL_RPM) why = "RPM TARGET";
        s.canShoot = why == null;
        s.whyNot = why == null ? "" : why;
        return s;
    }

    /** Piecewise-linear lookup, clamped at both ends. */
    public static double interpolate(double[] xs, double[] ys, double x) {
        int n = Math.min(xs.length, ys.length);
        if (n == 0) return 0.0;
        if (x <= xs[0]) return ys[0];
        for (int i = 1; i < n; i++) {
            if (x <= xs[i]) {
                double u = (x - xs[i - 1]) / Math.max(1e-9, xs[i] - xs[i - 1]);
                return ys[i - 1] + u * (ys[i] - ys[i - 1]);
            }
        }
        return ys[n - 1];
    }
}
