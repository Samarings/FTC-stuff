package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.biobuzz.BallType;
import org.firstinspires.ftc.teamcode.biobuzz.Buttons;
import org.firstinspires.ftc.teamcode.biobuzz.CellSide;
import org.firstinspires.ftc.teamcode.biobuzz.Feeder;
import org.firstinspires.ftc.teamcode.biobuzz.FieldDraw;
import org.firstinspires.ftc.teamcode.biobuzz.PoseStorage;
import org.firstinspires.ftc.teamcode.biobuzz.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConstants;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver;

/**
 * BIOBUZZ TeleOp.
 *
 * POSE: every INIT resets the Pinpoint and starts at x=0, y=0, heading 0. Point the robot the
 * way "forward" should be before INIT: field-centric drive uses that heading. Because (0,0) is
 * not a real field position, Limelight pose corrections are off and odometry-based aiming /
 * shot distance are not field-accurate; Limelight tx aiming still works when tags are visible.
 * Nothing is commanded until START (G403: no motion in the AUTO->TELEOP transition).
 *
 * INIT: X / B alliance (which tags are ours), gamepad 1 Y toggles PRACTICE mode (no 2:00
 * shutoff; default when no AUTO ran).
 *
 * GAMEPAD 1 (driver)
 *   left stick      field-centric translation (forward = the way the robot faced at INIT)
 *   right stick X   turn (manual)
 *   A               toggle AUTO-AIM: aim takes over rotation only, you keep translation
 *   Y               slow mode
 *   BACK + START    hold 1 s: reset pose to (0, 0, 0) (point the robot "forward" first).
 *
 * GAMEPAD 2 (operator)
 *   B               shooter on/off (on = solver RPM for the current distance)
 *   BACK            idle spin on/off when the shooter is off (default on)
 *   right trigger   FEEDER: spins the feeder forward, balls go up into the turret/flywheel.
 *                   Direct control (no aim/RPM check): spin the shooter up first (B).
 *   left trigger    intake
 *   dpad down       backtake / eject (reverse intake + feeder)
 *   dpad left/right RPM trim -/+ 50
 *   dpad up         we TIPPED but vision missed it: switch to the other CELL (+1 TIP)
 *   Y               ball type POLLEN / NECTAR (selects the RPM table)
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BiobuzzTeleOp extends OpMode {

    private Robot robot;
    private Follower follower;
    private TelemetryManager telemetryM;
    private final Buttons buttons = new Buttons();

    private Alliance alliance = Alliance.RED;
    private boolean fromAuto = false;

    private boolean aimOn = false;
    private boolean slow = false;
    private boolean shooterOn = false;
    private boolean idleOn = true;
    private double rpmTrim = 0.0;
    private BallType ballType = BallType.POLLEN;

    private double startSec = 0.0, lastSec = 0.0;
    private boolean matchOver = false;
    /** Practice: no 2:00 auto-shutoff. Defaults ON when no AUTO ran; G1 Y toggles in INIT. */
    private boolean practice = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        FieldDraw.init();

        fromAuto = PoseStorage.isFresh(600.0);
        if (fromAuto) alliance = PoseStorage.alliance;
        CellSide up = fromAuto && PoseStorage.upCell != null ? PoseStorage.upCell : alliance.startUpCell();

        robot = new Robot(hardwareMap, alliance, up, false);
        follower = robot.localizer.follower();
        if (fromAuto) robot.tips.setTipCount(PoseStorage.tipCount);
        // Always start at (0, 0, 0): reset the Pinpoint + IMU while the robot is still.
        robot.localizer.resetImuAndSetPose(ZERO_POSE);
        robot.feeder.forgetBallCount(); // no ball sensor
        practice = !fromAuto;
    }

    @Override
    public void init_loop() {
        if (!fromAuto) {
            boolean changed = false;
            if (buttons.pressed("x", gamepad1.x) && alliance != Alliance.BLUE) { alliance = Alliance.BLUE; changed = true; }
            if (buttons.pressed("b", gamepad1.b) && alliance != Alliance.RED) { alliance = Alliance.RED; changed = true; }
            if (changed) robot.configure(alliance, alliance.startUpCell());
        }
        if (buttons.pressed("inity", gamepad1.y)) practice = !practice;
        double now = getRuntime();
        robot.localizer.updatePoseOnly(now);
        robot.vision.update(robot.localizer.getHeadingDeg(), 0.0, now);

        telemetryM.addLine("=== BIOBUZZ TeleOp ===");
        telemetryM.addLine(practice
                ? "PRACTICE MODE (G1 Y): no 2:00 shutoff. Use MATCH mode in real matches!"
                : "MATCH MODE (G1 Y): all outputs stop 2:00 after START (G404).");
        telemetryM.addLine("Alliance" + (fromAuto ? " (from AUTO): " : " (X blue / B red): ") + alliance);
        telemetryM.addLine("Pose starts at (0, 0, 0): " + fmt(robot.localizer.getPose()));
        telemetryM.addLine("Point the robot the way you want FORWARD to be, then keep it still.");
        telemetryM.addLine("Limelight: " + (robot.vision.isConnected() ? "connected" : "NOT CONNECTED"));
        for (String w : RobotConstants.placeholderWarnings()) telemetryM.addLine("⚠ " + w);
        telemetryM.addLine("Nothing moves until START.");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        startSec = getRuntime();
        lastSec = startSec;
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double t = now - startSec;
        double dt = now - lastSec;
        if (dt <= 0 || dt > 0.2) dt = 0.02;
        lastSec = now;

        boolean periodOk = practice || t < TELEOP_LENGTH_S;
        // Pose starts at (0,0,0), not a field position: never apply Limelight pose corrections.
        ShotSolver.Solution sol = robot.sense(now, ballType, periodOk, rpmTrim, false);

        // G404: nothing powered after the match ends.
        if (!periodOk) {
            if (!matchOver) {
                matchOver = true;
                robot.stopMechanisms();
            }
            follower.setTeleOpDrive(0, 0, 0, true); // applied by next loop's follower.update()
            telemetryM.addLine("MATCH OVER: all outputs off (G404). Press STOP.");
            telemetryM.update(telemetry);
            return;
        }

        // ---------------- driver
        if (buttons.pressed("g1a", gamepad1.a)) { aimOn = !aimOn; robot.aimer.reset(); }
        if (buttons.pressed("g1y", gamepad1.y)) slow = !slow;
        if (buttons.heldFor("reseed", gamepad1.back && gamepad1.start, 1.0, now)) {
            robot.localizer.setPose(ZERO_POSE);
        }

        double scale = slow ? TELEOP_SLOW_SCALE : 1.0;
        // Field-centric: heading 0 = the way the robot faced at INIT, so stick up = that way.
        double fwd = -gamepad1.left_stick_y * scale;
        double strafe = -gamepad1.left_stick_x * scale;
        double turn = -gamepad1.right_stick_x * TELEOP_TURN_SCALE * scale;
        if (aimOn) turn = robot.aimer.update(sol, dt); // aim owns rotation only
        follower.setTeleOpDrive(fwd, strafe, turn, false);

        // ---------------- operator
        if (buttons.pressed("g2b", gamepad2.b)) shooterOn = !shooterOn;
        if (buttons.pressed("g2back", gamepad2.back)) idleOn = !idleOn;
        if (buttons.pressed("g2y", gamepad2.y)) ballType = ballType == BallType.POLLEN ? BallType.NECTAR : BallType.POLLEN;
        if (buttons.pressed("g2l", gamepad2.dpad_left)) rpmTrim -= TELEOP_RPM_TRIM_STEP;
        if (buttons.pressed("g2r", gamepad2.dpad_right)) rpmTrim += TELEOP_RPM_TRIM_STEP;
        if (buttons.pressed("g2up", gamepad2.dpad_up)) robot.tips.manualFlip(now, true);

        if (shooterOn) robot.shooter.setTargetRpm(sol.rpm);
        else if (idleOn) robot.shooter.idle();
        else robot.shooter.off();

        // Right trigger = feeder forward (into the turret), left trigger = intake, dpad down = backtake.
        double feederPwr = gamepad2.right_trigger > 0.05 ? gamepad2.right_trigger * FEED_FEEDER_POWER : 0.0;
        double intakePwr = gamepad2.left_trigger > 0.05 ? INTAKE_COLLECT_POWER : 0.0;
        if (gamepad2.dpad_down) robot.feeder.setMode(Feeder.Mode.EJECT);
        else if (feederPwr > 0 || intakePwr > 0) robot.feeder.setManual(intakePwr, feederPwr);
        else robot.feeder.setMode(Feeder.Mode.HOLD);

        robot.actuate(now, dt);
        robot.savePose();

        // ---------------- telemetry
        double left = Math.max(0, TELEOP_LENGTH_S - t);
        telemetryM.addLine(practice ? String.format("%s | PRACTICE  t=%.0f s", alliance, t)
                : String.format("%s | %d:%02d left%s", alliance, (int) left / 60, (int) left % 60,
                left <= 60 ? "  FLOWERS OPEN (NECTAR ok)" : "  (no NECTAR in FLOWERS yet: G410)"));
        telemetryM.addLine("AIM " + (aimOn ? "ON" : "off") + "  slow " + slow + "  " + robot.aimer.name());
        telemetryM.addLine(String.format("Target %s CELL  dist=%.1f in  aimErr=%.1f° (%s) tol=%.1f°",
                sol.upCell, sol.distanceIn, sol.aimErrDeg, sol.visionAim ? "LL" : "odom", sol.toleranceDeg));
        telemetryM.addLine(String.format("Shooter %s  RPM tgt/meas %.0f / %.0f  trim %+.0f  %s",
                shooterOn ? "ON" : (idleOn ? "idle" : "off"), robot.shooter.getTargetRpm(),
                robot.shooter.getMeasuredRpm(), rpmTrim, ballType));
        if (robot.shooter.getFault() != null)
            telemetryM.addLine("⚠ SHOOTER FAULT: " + robot.shooter.getFault() + "  (G2 B off clears it)");
        telemetryM.addLine(String.format("Feeder (RT) %.2f  Intake (LT) %.2f  %s", feederPwr, intakePwr,
                gamepad2.dpad_down ? "BACKTAKE" : ""));
        telemetryM.addLine("Aimed+ready (info only): " + (sol.canShoot ? "YES" : "no (" + sol.whyNot + ")"));
        telemetryM.addLine(String.format("TIPS %d  ~%d more to tip%s  shots %d",
                robot.tips.getTipCount(), robot.tips.ballsToTipEstimate(),
                robot.tips.tipLikely() ? " (TIP LIKELY)" : "", robot.shooter.getShotsDetected()));
        for (String l : robot.localizer.describe(now)) telemetryM.addLine(l);
        telemetryM.update(telemetry);
        FieldDraw.draw(robot.localizer.getPose(), robot.localizer.getLastAcceptedVisionPose(), sol.target);
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopMechanisms();
            robot.savePose();
            robot.vision.stop();
        }
    }

    private static final Pose ZERO_POSE = new Pose(0.0, 0.0, 0.0);

    private static String fmt(Pose p) {
        return p == null ? "none" : String.format("(%.1f, %.1f, %.0f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }
}
