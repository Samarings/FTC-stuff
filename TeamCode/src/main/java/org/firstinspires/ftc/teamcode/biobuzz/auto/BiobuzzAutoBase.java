package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.biobuzz.BallType;
import org.firstinspires.ftc.teamcode.biobuzz.Buttons;
import org.firstinspires.ftc.teamcode.biobuzz.CellSide;
import org.firstinspires.ftc.teamcode.biobuzz.Feeder;
import org.firstinspires.ftc.teamcode.biobuzz.Field;
import org.firstinspires.ftc.teamcode.biobuzz.FieldDraw;
import org.firstinspires.ftc.teamcode.biobuzz.PoseStorage;
import org.firstinspires.ftc.teamcode.biobuzz.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConstants;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Shared BIOBUZZ AUTO engine. Each route is a small subclass that lists its steps in
 * {@link #buildRoute()}. Routes are written for RED; blue is generated with rotate180().
 *
 * INIT menu (gamepad 1, shown on telemetry):
 *   X / B         alliance BLUE / RED
 *   dpad up/down  start position
 *   dpad l/r      park spot (lower / upper half of the LOADING ZONE)
 *   bumpers       start delay -/+ 0.5 s (0-10 s)
 *   Y             flip the starting up CELL (only if the field was set up differently)
 *   BACK          route option (if the route has one)
 *
 * Rules handled here:
 *   G304.H  nothing moves in INIT (Pinpoint/IMU reset and pose seeding only).
 *   G402    every waypoint is kept on our half; paths detour around the HIVE frame.
 *   G407    pre-loads = 4; collection is skipped unless the estimate says we're empty.
 *   G417    the feeder only runs while the ShotSolver says canShoot.
 *   Time    every step has a timeout; at AUTO_PARK_DEADLINE_S we drive to park no
 *           matter what; mechanisms stop at AUTO_MECH_STOP_S.
 */
public abstract class BiobuzzAutoBase extends OpMode {

    // ============================================================ route API for subclasses

    protected abstract String routeName();

    /** Add steps with the helpers below. Called at START (alliance is final by then). */
    protected abstract void buildRoute();

    /** Optional route setting cycled with gamepad1.back in INIT. */
    protected String optionLabel() { return null; }
    protected void cycleOption() {}

    /** Extra red-side poses to sanity-check in INIT (G402 / HIVE frame). */
    protected List<Pose> routePosesRed() { return new ArrayList<>(); }

    // ============================================================ state

    protected Robot robot;
    protected Follower follower;
    private TelemetryManager telemetryM;
    private final Buttons buttons = new Buttons();

    protected Alliance alliance = Alliance.RED;
    protected AutoPoses.Start startPos = AutoPoses.Start.ALLIANCE_WALL;
    protected boolean parkUpper = false;
    protected double startDelayS = 0.0;
    protected boolean flipUpCell = false;

    private final List<Step> steps = new ArrayList<>();
    private int stepIndex = -1;
    private Step current = null;
    private double stepStartSec = 0.0;
    private int parkIndex = -1;

    private double matchStartSec = 0.0;
    private double lastLoopSec = 0.0;
    private boolean mechStopped = false;
    private boolean deadlineJumped = false;
    private final List<String> planWarnings = new ArrayList<>();
    /** Feeder mode requested by the current step this loop (applied once, after the step). */
    protected Feeder.Mode want = Feeder.Mode.HOLD;

    // ============================================================ OpMode

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        FieldDraw.init();
        robot = new Robot(hardwareMap, alliance, initialUpCell(), true);
        follower = robot.localizer.follower();

        robot.shooter.off();
        robot.feeder.setMode(Feeder.Mode.HOLD);
        robot.feeder.setBallEstimate(PRELOAD_BALLS);

        // Robot is still: reset Pinpoint + IMU now (not after START like DECODE), seed the pose.
        robot.localizer.resetImuAndSetPose(startPose());
    }

    @Override
    public void init_loop() {
        boolean changed = false;
        if (buttons.pressed("x", gamepad1.x) && alliance != Alliance.BLUE) { alliance = Alliance.BLUE; changed = true; }
        if (buttons.pressed("b", gamepad1.b) && alliance != Alliance.RED) { alliance = Alliance.RED; changed = true; }
        if (buttons.pressed("up", gamepad1.dpad_up)) { startPos = cycle(startPos, 1); changed = true; }
        if (buttons.pressed("dn", gamepad1.dpad_down)) { startPos = cycle(startPos, -1); changed = true; }
        if (buttons.pressed("l", gamepad1.dpad_left)) parkUpper = false;
        if (buttons.pressed("r", gamepad1.dpad_right)) parkUpper = true;
        if (buttons.pressed("lb", gamepad1.left_bumper)) startDelayS = Math.max(0, startDelayS - 0.5);
        if (buttons.pressed("rb", gamepad1.right_bumper)) startDelayS = Math.min(10, startDelayS + 0.5);
        if (buttons.pressed("y", gamepad1.y)) { flipUpCell = !flipUpCell; changed = true; }
        if (buttons.pressed("back", gamepad1.back)) cycleOption();

        if (changed) {
            robot.configure(alliance, initialUpCell());
            robot.localizer.setPose(startPose()); // no IMU reset needed: heading is set too
        }

        double now = getRuntime();
        robot.localizer.updatePoseOnly(now);
        robot.vision.update(robot.localizer.getHeadingDeg(), 0.0, now);

        telemetryM.addLine("=== BIOBUZZ AUTO: " + routeName() + " ===");
        telemetryM.addLine("Alliance (X blue / B red): " + alliance);
        telemetryM.addLine("Start (dpad up/down): " + startPos);
        telemetryM.addLine("Park (dpad l/r): " + (parkUpper ? "UPPER" : "LOWER") + " half of LOADING ZONE");
        telemetryM.addLine(String.format("Start delay (bumpers): %.1f s", startDelayS));
        telemetryM.addLine("Up CELL at start (Y flips): " + initialUpCell() + (flipUpCell ? "  (FLIPPED!)" : ""));
        if (optionLabel() != null) telemetryM.addLine("Option (BACK): " + optionLabel());
        Pose p = robot.localizer.getPose();
        telemetryM.addLine(String.format("Seeded pose: x=%.1f y=%.1f h=%.1f°", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        telemetryM.addLine("Limelight: " + (robot.vision.isConnected() ? "connected" : "NOT CONNECTED")
                + "  tags: " + robot.vision.getVisibleIds());
        telemetryM.addLine("Aimer: " + robot.aimer.name());
        for (String w : validateStaticPoses()) telemetryM.addLine("⚠ " + w);
        for (String w : RobotConstants.placeholderWarnings()) telemetryM.addLine("⚠ " + w);
        telemetryM.addLine("LOCALIZER READY. Keep the robot still.");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        matchStartSec = getRuntime();
        lastLoopSec = matchStartSec;
        PoseStorage.clear();
        steps.clear();
        if (startDelayS > 0) steps.add(new WaitStep("start delay", startDelayS));
        buildRoute();
        parkIndex = -1;
        for (int i = 0; i < steps.size(); i++) if (steps.get(i).isPark) parkIndex = i;
        if (parkIndex < 0) {
            steps.add(park());
            parkIndex = steps.size() - 1;
        }
        advance(matchStartSec, 0);
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double t = now - matchStartSec;
        double dt = now - lastLoopSec;
        if (dt <= 0 || dt > 0.2) dt = 0.02;
        lastLoopSec = now;

        boolean pathRunning = follower.isBusy();
        ShotSolver.Solution sol = robot.sense(now, BallType.POLLEN, t < 30.0, 0.0, !pathRunning);

        // Flywheel stays spinning the whole AUTO (no braking between volleys).
        if (!mechStopped) robot.shooter.setTargetRpm(sol.rpm);
        want = Feeder.Mode.HOLD; // steps override

        // Hard deadline: go park wherever we are.
        if (!deadlineJumped && t >= AUTO_PARK_DEADLINE_S && current != null && !current.isPark) {
            deadlineJumped = true;
            jumpTo(parkIndex, now);
        }

        if (current != null) {
            boolean done = current.update(now, dt, sol);
            boolean timedOut = now - stepStartSec >= current.timeoutS;
            if (done || timedOut) {
                if (timedOut && !done) planWarnings.add("timeout: " + current.name);
                current.end();
                advance(now, stepIndex + 1);
            }
        }
        // (DONE: the last path was followed with holdEnd = true, so Pedro keeps us parked.)
        if (!mechStopped) robot.feeder.setMode(want);

        if (t >= AUTO_MECH_STOP_S && !mechStopped) {
            mechStopped = true;
            robot.stopMechanisms();
        }
        if (!mechStopped) robot.actuate(now, dt);

        robot.savePose();
        telemetry(sol, t);
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopMechanisms();
            robot.savePose();
            robot.vision.stop();
        }
    }

    // ============================================================ step plumbing

    private void advance(double now, int index) {
        while (index < steps.size() && steps.get(index).skip()) index++;
        if (index >= steps.size()) {
            current = null;
            stepIndex = steps.size();
            return;
        }
        stepIndex = index;
        current = steps.get(index);
        stepStartSec = now;
        current.begin(now);
    }

    private void jumpTo(int index, double now) {
        if (current != null) current.end();
        advance(now, index);
    }

    /** One AUTO action. Return true from update() when finished. */
    protected abstract class Step {
        final String name;
        final double timeoutS;
        boolean isPark = false;
        BooleanSupplier skipIf = null;

        Step(String name, double timeoutS) {
            this.name = name;
            this.timeoutS = timeoutS;
        }

        boolean skip() { return skipIf != null && skipIf.getAsBoolean(); }
        void begin(double now) {}
        abstract boolean update(double now, double dt, ShotSolver.Solution sol);
        void end() {}

        /** Skip this step when the condition is true at the moment it would start. */
        public Step skipWhen(BooleanSupplier cond) {
            skipIf = cond;
            return this;
        }
    }

    protected class WaitStep extends Step {
        WaitStep(String name, double seconds) { super(name, seconds); }
        @Override boolean update(double now, double dt, ShotSolver.Solution sol) { return false; }
    }

    /**
     * Drive to a red-side pose (converted to our alliance), detouring around the HIVE frame.
     * faceTarget: end heading = facing our up CELL (computed, never hand-typed).
     * Fires en route whenever canShoot (unless intaking).
     */
    protected class DriveStep extends Step {
        private final Supplier<Pose> redTarget;
        private final boolean faceTarget, intake;
        private final double maxPower;

        DriveStep(String name, Supplier<Pose> redTarget, boolean faceTarget, boolean intake, double maxPower) {
            super(name, AUTO_DRIVE_STEP_TIMEOUT_S);
            this.redTarget = redTarget;
            this.faceTarget = faceTarget;
            this.intake = intake;
            this.maxPower = maxPower;
        }

        @Override
        void begin(double now) {
            Pose target = alliance.fromRed(redTarget.get());
            double endH = target.getHeading();
            if (faceTarget) {
                Pose cell = Field.cellPosition(alliance, robot.tips.getUpCell());
                Pose axisAtTarget = Field.turretAxis(target);
                endH = Field.headingToward(axisAtTarget.getX(), axisAtTarget.getY(), cell.getX(), cell.getY());
            }
            follower.setMaxPower(maxPower);
            followRoute(robot.localizer.getPose(), new Pose(target.getX(), target.getY(), endH), name);
        }

        @Override
        boolean update(double now, double dt, ShotSolver.Solution sol) {
            if (intake) want = Feeder.Mode.COLLECT;
            else if (sol.canShoot) want = Feeder.Mode.FEED;
            return !follower.isBusy();
        }

        @Override
        void end() { follower.setMaxPower(1.0); }
    }

    /**
     * Hold position, face the up CELL (odometry heading, fine-corrected by Limelight tx) and
     * fire whenever canShoot. Ends when the ball estimate reaches 0 (plus a short tail so the
     * last ball clears) or on timeout.
     */
    protected class ShootStep extends Step {
        private Pose holdXY;
        private double holdHeading, issuedHeading;
        private double emptySince = -1;

        ShootStep(String name, double timeoutS) { super(name, timeoutS); }

        @Override
        void begin(double now) {
            holdXY = robot.localizer.getPose();
            holdHeading = robot.solution != null ? robot.solution.desiredHeadingRad : holdXY.getHeading();
            issue();
            emptySince = -1;
        }

        private void issue() {
            issuedHeading = holdHeading;
            follower.holdPoint(new Pose(holdXY.getX(), holdXY.getY(), holdHeading));
        }

        @Override
        boolean update(double now, double dt, ShotSolver.Solution sol) {
            double heading = robot.localizer.getPose().getHeading();
            double goal = sol.visionAim ? heading + Math.toRadians(sol.aimErrDeg) : sol.desiredHeadingRad;
            double maxStep = Math.toRadians(AUTO_TX_CORRECT_DEG_S) * dt;
            double d = Field.wrapRad(goal - holdHeading);
            holdHeading = Field.wrapRad(holdHeading + Math.max(-maxStep, Math.min(maxStep, d)));
            if (!sol.visionAim) holdHeading = goal; // odometry target is exact; no need to rate-limit
            if (Math.abs(Math.toDegrees(Field.wrapRad(holdHeading - issuedHeading))) > AUTO_REHOLD_DEG) issue();

            want = sol.canShoot ? Feeder.Mode.FEED : Feeder.Mode.HOLD;

            if (robot.feeder.isBallCountKnown() && robot.feeder.getBallEstimate() == 0) {
                if (emptySince < 0) emptySince = now;
                return now - emptySince > 0.3;
            }
            return false;
        }
    }

    /** Run the intake while holding still, then assume the source gave us a full load. */
    protected class IntakeDwellStep extends Step {
        IntakeDwellStep(String name, double seconds) { super(name, seconds + 0.5); this.seconds = seconds; }
        private final double seconds;
        private double start;

        @Override void begin(double now) { start = now; }

        @Override
        boolean update(double now, double dt, ShotSolver.Solution sol) {
            want = Feeder.Mode.COLLECT;
            return now - start >= seconds;
        }

        @Override
        void end() {
            // No ball sensor: assume the source's balls are on board (we were empty before).
            robot.feeder.setBallEstimate(MAX_BALLS);
        }
    }

    // ============================================================ helpers for routes

    protected Step driveToShootSpot() {
        return new DriveStep("drive to shoot spot",
                () -> AutoPoses.shootSpot(redFrameUpCell()), true, false, 1.0);
    }

    protected Step shoot(double timeoutS) {
        return new ShootStep("shoot", timeoutS);
    }

    /** Second shooting attempt if the estimate says balls are left (missed shot detection). */
    protected Step shootRetry() {
        return new ShootStep("shoot (retry)", 2.0).skipWhen(this::isEmpty);
    }

    protected Step driveTo(String name, Supplier<Pose> redPose, boolean intake, double power) {
        return new DriveStep(name, redPose, false, intake, power);
    }

    protected Step intakeDwell(double seconds) {
        return new IntakeDwellStep("intake", seconds);
    }

    protected Step park() {
        Step s = new DriveStep("park", () -> AutoPoses.park(parkUpper), false, false, 1.0);
        s.isPark = true;
        return s;
    }

    protected void add(Step s) { steps.add(s); }

    /** True if we still hold balls: collection steps are skipped then (G407, no sensor). */
    protected boolean notEmpty() {
        return !robot.feeder.isBallCountKnown() || robot.feeder.getBallEstimate() > 0;
    }

    protected boolean isEmpty() { return !notEmpty(); }

    /** Our up CELL expressed as the matching red-side CELL (blue's FAR = red's AUDIENCE twin). */
    protected CellSide redFrameUpCell() {
        CellSide up = robot.tips.getUpCell();
        return alliance == Alliance.RED ? up : up.other();
    }

    // ============================================================ path planning

    /** Follow straight segments to the goal, detouring via the lane if the HIVE frame is in the way. */
    private void followRoute(Pose from, Pose goal, String label) {
        // Plan in the red frame (the field is symmetric), then convert back.
        Pose fromR = alliance.fromRed(from);       // rotate180 is its own inverse
        Pose goalR = alliance.fromRed(goal);
        List<Pose> wpsR = plan(fromR, goalR, label);

        double startH = from.getHeading();
        double endH = startH + Field.wrapRad(goal.getHeading() - startH); // shortest turn

        PathBuilder b = follower.pathBuilder();
        Pose prev = from;
        for (int i = 0; i < wpsR.size(); i++) {
            Pose wp = alliance.fromRed(wpsR.get(i));
            wp = new Pose(wp.getX(), wp.getY(), endH);
            b.addPath(new BezierLine(prev, wp));
            if (i == 0) b.setLinearHeadingInterpolation(startH, endH);
            else b.setConstantHeadingInterpolation(endH);
            prev = wp;
        }
        follower.followPath(b.build(), true);
    }

    private List<Pose> plan(Pose fromR, Pose goalR, String label) {
        goalR = clampToOurHalfRed(goalR, label);
        List<Pose> out = new ArrayList<>();
        if (!Field.segmentHitsHive(fromR, goalR)) {
            out.add(goalR);
            return out;
        }
        double lane = Math.min(AutoPoses.LANE_X, Field.CENTER_IN - 12);
        Pose a = new Pose(lane, fromR.getY(), 0);
        Pose b = new Pose(lane, goalR.getY(), 0);
        if (!Field.segmentHitsHive(fromR, a) && !Field.segmentHitsHive(a, b) && !Field.segmentHitsHive(b, goalR)) {
            if (Math.abs(fromR.getX() - lane) > 1.0) out.add(a);
            out.add(b);
            out.add(goalR);
            return out;
        }
        planWarnings.add("PATH NEAR HIVE: " + label);
        out.add(goalR);
        return out;
    }

    private Pose clampToOurHalfRed(Pose p, String label) {
        double margin = Math.max(ROBOT_HALF_LENGTH_IN, ROBOT_HALF_WIDTH_IN);
        double maxX = Field.CENTER_IN - margin;
        if (p.getX() > maxX) {
            planWarnings.add("G402 clamp: " + label);
            return new Pose(maxX, p.getY(), p.getHeading());
        }
        return p;
    }

    private List<String> validateStaticPoses() {
        List<String> w = new ArrayList<>();
        List<Pose> poses = new ArrayList<>();
        poses.add(AutoPoses.start(startPos));
        poses.add(AutoPoses.shootSpot(CellSide.AUDIENCE));
        poses.add(AutoPoses.shootSpot(CellSide.FAR));
        poses.add(AutoPoses.park(false));
        poses.add(AutoPoses.park(true));
        poses.addAll(routePosesRed());
        for (Pose p : poses) {
            if (!Field.onOurHalf(Alliance.RED, p.getX()))
                w.add(String.format("pose (%.0f,%.0f) crosses x=72 (G402)", p.getX(), p.getY()));
            double inflate = Math.max(ROBOT_HALF_LENGTH_IN, ROBOT_HALF_WIDTH_IN);
            if (Field.insideHiveFrame(p.getX(), p.getY(), inflate))
                w.add(String.format("pose (%.0f,%.0f) overlaps the HIVE frame", p.getX(), p.getY()));
        }
        return w;
    }

    // ============================================================ misc

    private CellSide initialUpCell() {
        CellSide c = alliance.startUpCell();
        return flipUpCell ? c.other() : c;
    }

    private Pose startPose() {
        return alliance.fromRed(AutoPoses.start(startPos));
    }

    private static AutoPoses.Start cycle(AutoPoses.Start s, int dir) {
        AutoPoses.Start[] v = AutoPoses.Start.values();
        return v[((s.ordinal() + dir) % v.length + v.length) % v.length];
    }

    private void telemetry(ShotSolver.Solution sol, double t) {
        telemetryM.addLine(String.format("%s | %s | t=%.1f s", routeName(), alliance, t));
        telemetryM.addLine("Step: " + (current == null ? "DONE (parked)" : current.name)
                + (deadlineJumped ? "  [PARK DEADLINE]" : ""));
        for (String l : robot.localizer.describe(getRuntime())) telemetryM.addLine(l);
        telemetryM.addLine(String.format("Target %s CELL  dist=%.1f in  aimErr=%.1f° (%s) tol=%.1f°",
                sol.upCell, sol.distanceIn, sol.aimErrDeg, sol.visionAim ? "LL" : "odom", sol.toleranceDeg));
        telemetryM.addLine(String.format("RPM tgt/meas %.0f / %.0f  ready=%b",
                robot.shooter.getTargetRpm(), robot.shooter.getMeasuredRpm(), robot.shooter.isReady()));
        telemetryM.addLine("canShoot: " + (sol.canShoot ? "YES" : "no (" + sol.whyNot + ")"));
        telemetryM.addLine(String.format("Balls (est) %d  shots %d  TIPS %d  ~%d more to tip%s",
                robot.feeder.getBallEstimate(), robot.shooter.getShotsDetected(), robot.tips.getTipCount(),
                robot.tips.ballsToTipEstimate(), robot.tips.tipLikely() ? " (TIP LIKELY)" : ""));
        telemetryM.addLine("Feeder: " + robot.feeder.getMode() + "  tags: " + robot.vision.getVisibleIds());
        for (String w : planWarnings) telemetryM.addLine("⚠ " + w);
        telemetryM.update(telemetry);
        FieldDraw.draw(robot.localizer.getPose(), robot.localizer.getLastAcceptedVisionPose(), sol.target);
    }
}
