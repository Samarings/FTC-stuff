package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.geometry.Pose;

import java.util.ArrayDeque;

/**
 * "Shoot from anywhere": one solver shared by AUTO and TeleOp, run every loop.
 *
 * 1. Target = our up CELL's opening (TipTracker says which CELL).
 * 2. Odometry feed-forward: field direction from the turret axis to the target point.
 * 3. Limelight correction: when 2+ tags of our up CELL's cluster are visible, the direction
 *    the camera actually saw the cluster center in (at the frame's capture time) wins; the
 *    difference to the odometry direction is remembered, so odometry carries the aim on when
 *    the tags drop out (too close, blocked, mid-flip).
 * 4. Distance (turret axis -> target, horizontal) -> RPM from the POLLEN or NECTAR table.
 * 5. canShoot gate. The feeder only runs while it's true: this is how we satisfy G417.
 *
 * Without a field pose (TeleOp starts at 0,0,0) the target point is not taken from the field
 * map: it is placed where the tags are seen (direction + Limelight distance) in the odometry
 * frame, and odometry keeps the aim on that point while the robot drives.
 *
 * The output is a FIELD direction for the shooter. TurretAimer points the turret that way
 * (through Turret.chooseSetpoint); ChassisAimer (TURRET_FIXED) turns the robot that way.
 */
public class ShotSolver {

    public static class Solution {
        public CellSide upCell;
        public Pose target;                 // aim point (x, y) in the pose frame
        public double distanceIn;
        /** False = no field pose and our tags haven't been seen yet: nothing to aim at. */
        public boolean hasAim;
        public double desiredHeadingRad;    // field direction that points the shooter at the target
        public double turretYawDeg;         // the same direction relative to the robot (-180..180)
        public boolean visionAim;           // the Limelight corrected the aim this loop
        public double aimErrDeg;            // desired - current shooter direction (+ = turn CCW)
        public double toleranceDeg;
        public double rpm;
        public boolean aimed;
        public boolean canShoot;
        public String whyNot = "";
    }

    private final Alliance alliance;

    private final ArrayDeque<double[]> history = new ArrayDeque<>(); // {timeSec, shooter field direction (deg)}
    private double corrDeg = 0.0;       // filtered (seen direction - odometry direction)
    private boolean corrValid = false;
    private double[] seenTarget = null; // {x, y} where the tags put the target (used without a field pose)
    private double aimX, aimY;          // target point used this loop
    private CellSide aimCell = null;
    private int aimedLoops = 0;

    public ShotSolver(Alliance alliance) {
        this.alliance = alliance;
    }

    /**
     * @param turretAngleDeg     turret angle from home (0 while TURRET_FIXED)
     * @param fieldPose          false when the pose is not a real field position (TeleOp starts
     *                           at 0,0,0): the target is located from the tags, not the field map
     * @param ballsHeld          estimated balls on board, or -1 if unknown (TeleOp, no sensor)
     * @param periodAllowsShots  false during the AUTO->TELEOP transition, after the match, etc.
     * @param rpmTrim            driver trim added to the table RPM (0 in AUTO)
     */
    public Solution solve(Localizer loc, Vision vision, TipTracker tips, Shooter shooter,
                          double turretAngleDeg, boolean fieldPose,
                          BallType ball, int ballsHeld, boolean periodAllowsShots,
                          double rpmTrim, double nowSec) {
        Solution s = new Solution();
        Pose robot = loc.getPose();
        s.upCell = tips.getUpCell();
        Pose mapCell = Field.cellPosition(alliance, s.upCell);
        Pose axis = Field.turretAxis(robot);
        double headingDeg = Math.toDegrees(robot.getHeading());

        // ---- aim: odometry direction + Limelight correction (our up CELL's cluster only)
        if (s.upCell != aimCell) { // after a TIP everything learned from the tags is for the other CELL
            corrValid = false;
            seenTarget = null;
            aimCell = s.upCell;
        }
        Vision.ClusterView cv = vision.ourCluster(s.upCell);
        s.visionAim = cv.aimable() && vision.getStalenessMs() <= VISION_MAX_STALENESS_MS;
        double bearing = aimDirectionDeg(nowSec, axis.getX(), axis.getY(), headingDeg + turretAngleDeg,
                fieldPose ? mapCell : null, s.visionAim, cv.txDeg, cv.tyDeg, cv.distanceIn, vision.getCaptureTimeSec());
        s.hasAim = !Double.isNaN(bearing);
        s.target = s.hasAim ? new Pose(aimX, aimY, 0) : mapCell;
        double distNow = Math.hypot(s.target.getX() - axis.getX(), s.target.getY() - axis.getY());
        if (s.hasAim) {
            if (SHOOT_WHILE_MOVING) {
                // v2: lead the target by our own velocity over the time of flight
                double lead = Field.headingToward(axis.getX(), axis.getY(),
                        aimX - loc.getVelX() * TIME_OF_FLIGHT_S, aimY - loc.getVelY() * TIME_OF_FLIGHT_S);
                bearing += Math.toDegrees(Field.wrapRad(lead - Field.headingToward(axis.getX(), axis.getY(), aimX, aimY)));
            }
            s.desiredHeadingRad = Field.wrapRad(Math.toRadians(bearing));
            s.turretYawDeg = Field.wrapDeg(bearing - headingDeg);
            s.aimErrDeg = Field.wrapDeg(bearing - headingDeg - turretAngleDeg);
        } else {
            s.desiredHeadingRad = Field.wrapRad(Math.toRadians(headingDeg + turretAngleDeg)); // stay as we are
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

        // ---- aim tolerance: half the usable opening width at this range, held for a few loops
        s.toleranceDeg = Math.toDegrees(Math.atan2(CELL_AIM_HALF_WIDTH_IN, Math.max(1.0, dist)));
        aimedLoops = s.hasAim && Math.abs(s.aimErrDeg) <= s.toleranceDeg ? aimedLoops + 1 : 0;
        s.aimed = aimedLoops >= AIMED_LOOPS;

        // ---- canShoot (first failing reason is reported)
        String why = null;
        if (Double.isNaN(robot.getX()) || Double.isNaN(robot.getY())) why = "POSE NaN";
        else if (!periodAllowsShots) why = "MATCH PERIOD";
        else if (tips.inSettleWindow(nowSec)) why = "TIP SETTLING";
        else if (ballsHeld == 0) why = "NO BALLS";
        else if (!s.hasAim) why = "NO TARGET (tags not seen yet)";
        else if (dist < MIN_SHOT_DIST_IN) why = "TOO CLOSE";
        else if (fieldPose && Field.insideHiveFrame(robot.getX(), robot.getY(), 0)) why = "UNDER HIVE";
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

    /**
     * The field direction (deg, pose frame) the shooter should point. Call once per loop.
     *
     * This is not a PID on tx (the camera is slow and late). Each usable frame says "at the
     * capture time the cluster center was camYaw to the right of where the shooter pointed
     * THEN", which gives the target's direction no matter how the turret or robot turned
     * since. The filtered difference to the odometry direction is kept and added every loop.
     *
     * @param axisX,axisY    turret axis position (pose frame, inches)
     * @param shooterDirDeg  where the shooter/camera points now: robot heading + turret angle
     * @param mapTarget      target position from the field map, or null without a field pose
     *                       (then the target is placed where the tags are seen)
     * @param tagsOk         2+ of our up CELL's tags in a fresh frame
     * @param txDeg,tyDeg    cluster center in that frame (+ = right / up of the crosshair)
     * @param distIn         Limelight distance to the cluster, inches (NaN if unknown)
     * @return NaN if there is nothing to aim at (no field pose and no tags seen yet);
     *         aimX/aimY hold the target point used
     */
    double aimDirectionDeg(double nowSec, double axisX, double axisY, double shooterDirDeg, Pose mapTarget,
                           boolean tagsOk, double txDeg, double tyDeg, double distIn, double captureTimeSec) {
        history.addLast(new double[]{nowSec, shooterDirDeg});
        while (nowSec - history.peekFirst()[0] > ODOM_HISTORY_S) history.removeFirst();

        double seenDeg = Double.NaN;
        if (tagsOk) {
            // The camera is pitched up, so tx is not the yaw on the floor plane: un-pitch it.
            double pitch = Math.toRadians(CAMERA_PITCH_DEG);
            double camYawDeg = Math.toDegrees(Math.atan2(Math.tan(Math.toRadians(txDeg)),
                    Math.cos(pitch) - Math.tan(Math.toRadians(tyDeg)) * Math.sin(pitch)));
            // shortcut: ignores the lens sitting 0.094 m in front of the yaw axis and our own
            // travel during the ~50 ms latency (both < 1° when slow, gone as tx -> 0); model
            // them if the aim lags while driving fast.
            seenDeg = shooterDirAt(captureTimeSec) - camYawDeg + AIM_TX_OFFSET_DEG; // tx + = right = clockwise
            if (mapTarget == null && !Double.isNaN(distIn)) {
                double mx = axisX + distIn * Math.cos(Math.toRadians(seenDeg));
                double my = axisY + distIn * Math.sin(Math.toRadians(seenDeg));
                if (seenTarget == null) seenTarget = new double[]{mx, my};
                else {
                    seenTarget[0] += TX_EMA_ALPHA * (mx - seenTarget[0]);
                    seenTarget[1] += TX_EMA_ALPHA * (my - seenTarget[1]);
                }
            }
        }
        if (mapTarget != null) { aimX = mapTarget.getX(); aimY = mapTarget.getY(); }
        else if (seenTarget != null) { aimX = seenTarget[0]; aimY = seenTarget[1]; }
        else return Double.NaN;

        double odomDeg = Math.toDegrees(Field.headingToward(axisX, axisY, aimX, aimY));
        if (tagsOk) {
            double c = Field.wrapDeg(seenDeg - odomDeg);
            corrDeg = corrValid ? corrDeg + TX_EMA_ALPHA * Field.wrapDeg(c - corrDeg) : c;
            corrValid = true;
        }
        return odomDeg + (corrValid ? corrDeg : 0.0);
    }

    /** Where the shooter pointed at a past time (deg); the ends of the history clamp. */
    private double shooterDirAt(double t) {
        double[] prev = null;
        for (double[] h : history) {
            if (h[0] >= t) {
                if (prev == null) return h[1];
                double u = (t - prev[0]) / Math.max(1e-6, h[0] - prev[0]);
                return prev[1] + u * Field.wrapDeg(h[1] - prev[1]);
            }
            prev = h;
        }
        return prev[1];
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
