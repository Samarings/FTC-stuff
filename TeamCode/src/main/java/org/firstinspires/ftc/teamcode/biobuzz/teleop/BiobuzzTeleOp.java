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
import org.firstinspires.ftc.teamcode.biobuzz.auto.AutoPoses;

/**
 * BIOBUZZ TeleOp. Starts from the AUTO pose (PoseStorage) without resetting the Pinpoint.
 * If no AUTO ran (AUTO isn't in use yet), the drive team picks the spot where they placed the
 * robot in INIT (X/B alliance, dpad up/down start spot) and the Pinpoint is reset there.
 * Nothing is commanded until START (G403: no motion in the AUTO->TELEOP transition).
 *
 * INIT: gamepad 1 Y toggles PRACTICE mode (no 2:00 shutoff; default when no AUTO ran).
 *
 * GAMEPAD 1 (driver)
 *   left stick      field-centric translation (from our alliance wall's point of view)
 *   right stick X   turn (manual)
 *   A               toggle AUTO-AIM: aim takes over rotation only, you keep translation
 *   Y               slow mode
 *   BACK + START    hold 1 s: re-seed pose. Square the robot into our alliance-wall /
 *                   far-wall corner first (back on the alliance wall, side on the far wall).
 *
 * GAMEPAD 2 (operator)
 *   B               shooter on/off (on = solver RPM for the current distance)
 *   BACK            idle spin on/off when the shooter is off (default on)
 *   right trigger   FIRE: feeds only while canShoot (aimed + at speed + valid target)
 *   left bumper     hold with FIRE: skip the aim check (you aim by hand). Still needs RPM.
 *   left trigger    intake
 *   dpad down       eject (reverse intake + feeder)
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
    private AutoPoses.Start startSpot = AutoPoses.Start.ALLIANCE_WALL;

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
        if (fromAuto) {
            robot.tips.setTipCount(PoseStorage.tipCount);
            robot.localizer.setPose(PoseStorage.pose); // no Pinpoint reset
        } else {
            // No AUTO ran (practice, or AUTO not in use yet): the robot is placed by hand on
            // a start spot. Reset the Pinpoint + IMU while it's still (nothing moves).
            robot.localizer.resetImuAndSetPose(defaultPose());
        }
        robot.feeder.forgetBallCount(); // no ball sensor
        practice = !fromAuto;
    }

    @Override
    public void init_loop() {
        if (!fromAuto) {
            boolean changed = false;
            if (buttons.pressed("x", gamepad1.x) && alliance != Alliance.BLUE) { alliance = Alliance.BLUE; changed = true; }
            if (buttons.pressed("b", gamepad1.b) && alliance != Alliance.RED) { alliance = Alliance.RED; changed = true; }
            if (buttons.pressed("up", gamepad1.dpad_up)) { startSpot = cycleSpot(startSpot, 1); changed = true; }
            if (buttons.pressed("dn", gamepad1.dpad_down)) { startSpot = cycleSpot(startSpot, -1); changed = true; }
            if (changed) {
                robot.configure(alliance, alliance.startUpCell());
                robot.localizer.setPose(defaultPose());
            }
        }
        if (buttons.pressed("inity", gamepad1.y)) practice = !practice;
        double now = getRuntime();
        robot.localizer.updatePoseOnly(now);
        robot.vision.update(robot.localizer.getHeadingDeg(), 0.0, now);

        telemetryM.addLine("=== BIOBUZZ TeleOp ===");
        telemetryM.addLine(practice
                ? "PRACTICE MODE (G1 Y): no 2:00 shutoff. Use MATCH mode in real matches!"
                : "MATCH MODE (G1 Y): all outputs stop 2:00 after START (G404).");
        if (fromAuto) {
            telemetryM.addLine("Pose from AUTO: " + fmt(PoseStorage.pose) + "  alliance " + alliance
                    + "  up CELL " + robot.tips.getUpCell());
        } else {
            telemetryM.addLine("No AUTO pose: robot placed by hand.");
            telemetryM.addLine("Alliance (X blue / B red): " + alliance);
            telemetryM.addLine("Start spot (dpad up/down): " + startSpot + "  " + fmt(defaultPose()));
            telemetryM.addLine("Put the robot on that spot (heading matters most). Quick test: just");
            telemetryM.addLine("face the robot AWAY from the drivers; field-centric uses this heading.");
            telemetryM.addLine("Or re-seed after START: G1 BACK+START in the alliance/far-wall corner.");
        }
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
        ShotSolver.Solution sol = robot.sense(now, ballType, periodOk, rpmTrim, true);

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
            robot.localizer.setPose(reseedPose());
        }

        double scale = slow ? TELEOP_SLOW_SCALE : 1.0;
        // Field-centric from our alliance wall: stick up = away from our wall.
        double sign = alliance == Alliance.RED ? 1.0 : -1.0;
        double fwd = -gamepad1.left_stick_y * scale * sign;   // field +x for red
        double strafe = -gamepad1.left_stick_x * scale * sign; // field +y for red
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

        boolean fire = gamepad2.right_trigger > 0.5;
        boolean manualAim = gamepad2.left_bumper;
        boolean manualOk = shooterOn && robot.shooter.isReady() && !robot.tips.inSettleWindow(now);
        Feeder.Mode mode = Feeder.Mode.HOLD;
        if (gamepad2.dpad_down) mode = Feeder.Mode.EJECT;
        else if (fire && (sol.canShoot || (manualAim && manualOk))) mode = Feeder.Mode.FEED;
        else if (gamepad2.left_trigger > 0.3) mode = Feeder.Mode.COLLECT;
        robot.feeder.setMode(mode);

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
        telemetryM.addLine("canShoot: " + (sol.canShoot ? "YES" : "no (" + sol.whyNot + ")")
                + (manualAim ? "  [MANUAL AIM]" : ""));
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

    private Pose defaultPose() {
        return alliance.fromRed(AutoPoses.start(startSpot));
    }

    private static AutoPoses.Start cycleSpot(AutoPoses.Start s, int dir) {
        AutoPoses.Start[] v = AutoPoses.Start.values();
        return v[((s.ordinal() + dir) % v.length + v.length) % v.length];
    }

    /** Back on our alliance wall, side on the far wall, facing away from our wall. */
    private Pose reseedPose() {
        return alliance.fromRed(new Pose(ROBOT_HALF_LENGTH_IN, 144.0 - ROBOT_HALF_WIDTH_IN, 0.0));
    }

    private static String fmt(Pose p) {
        return p == null ? "none" : String.format("(%.1f, %.1f, %.0f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }
}
