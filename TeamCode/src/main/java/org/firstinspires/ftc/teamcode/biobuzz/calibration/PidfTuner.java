package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.biobuzz.Buttons;
import org.firstinspires.ftc.teamcode.biobuzz.Feeder;
import org.firstinspires.ftc.teamcode.biobuzz.HeadingHold;
import org.firstinspires.ftc.teamcode.biobuzz.PidfStore;
import org.firstinspires.ftc.teamcode.biobuzz.Robot;

/**
 * Tune the SHOOTER and HEADING HOLD PIDF gains with the gamepad.
 *
 * GAMEPAD 2 (tuning)
 *   dpad up / down     select a gain
 *   dpad right / left  increase / decrease the selected gain by one step
 *   right / left bumper  step x10 / ÷10
 *   B                  shooter on / off (closed-loop at the test RPM)
 *   Y / X              test RPM + / - RPM_ADJUST_STEP (50)
 *   right trigger      feeder (shoot a ball to see the RPM dip and recovery)
 *   A                  SAVE gains to the Control Hub (loaded by every OpMode)
 *   BACK (hold 1 s)    delete the saved file, back to the RobotConstants values
 *
 * GAMEPAD 1 (driving + heading test)
 *   sticks             drive (field-centric, forward = the way the robot faced at INIT)
 *   A                  heading hold on / off
 *   dpad left / right  step test: hold target +90° / -90° (watch overshoot and settling)
 *   push the robot by hand while holding: it should come back without oscillating
 *
 * Shooter tuning order: KF first (KP=KI=KD=0; raise KF until measured RPM ≈ target),
 * then KP until recovery after a shot is quick without oscillating, small KD if it
 * overshoots. KI usually stays 0. Heading: KP until it holds firmly, KD to stop the
 * wobble, KF only if small errors don't move the robot.
 */
@TeleOp(name = "PIDF Tuner", group = "Calibration")
public class PidfTuner extends OpMode {
    private static final double[] BASE_STEP = {
            0.001, 0.0001, 0.00001, 0.00001,   // SHOOTER_KP, KI, KD, KF
            0.002, 0.001, 0.0002, 0.01,        // HOLD_KP, KI, KD, KF
    };

    private Robot robot;
    private Follower follower;
    private TelemetryManager telemetryM;
    private final Buttons buttons = new Buttons();
    private final HeadingHold hold = new HeadingHold();

    private int sel = 0;
    private int stepExp = 0; // step = BASE_STEP * 10^stepExp
    private boolean shooterOn = false, holdOn = true;
    private double testRpm = 2500.0;
    private double lastSec = -1;
    private String message = "";

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Robot(hardwareMap, Alliance.RED, Alliance.RED.startUpCell(), false);
        follower = robot.localizer.follower();
        robot.localizer.resetImuAndSetPose(new Pose(0, 0, 0));
    }

    @Override
    public void init_loop() {
        telemetry.addLine("PIDF Tuner: point the robot FORWARD, keep it still, press START.");
        telemetry.addLine("Gains: " + PidfStore.status());
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lastSec = getRuntime();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = Math.max(1e-3, Math.min(0.2, now - lastSec));
        lastSec = now;
        robot.localizer.update(now);

        // ---------------- gain editing (gamepad 2)
        int n = PidfStore.KEYS.length;
        if (buttons.pressed("up", gamepad2.dpad_up)) sel = (sel + n - 1) % n;
        if (buttons.pressed("dn", gamepad2.dpad_down)) sel = (sel + 1) % n;
        if (buttons.pressed("rb", gamepad2.right_bumper)) stepExp = Math.min(3, stepExp + 1);
        if (buttons.pressed("lb", gamepad2.left_bumper)) stepExp = Math.max(-3, stepExp - 1);
        double step = BASE_STEP[sel] * Math.pow(10, stepExp);
        String key = PidfStore.KEYS[sel];
        if (buttons.pressed("r", gamepad2.dpad_right)) PidfStore.set(key, round(PidfStore.get(key) + step));
        if (buttons.pressed("l", gamepad2.dpad_left)) PidfStore.set(key, Math.max(0.0, round(PidfStore.get(key) - step)));
        if (buttons.pressed("a", gamepad2.a)) message = PidfStore.save();
        if (buttons.heldFor("revert", gamepad2.back, 1.0, now)) message = PidfStore.revertToCode();

        // ---------------- shooter
        if (buttons.pressed("b", gamepad2.b)) shooterOn = !shooterOn;
        if (buttons.pressed("y", gamepad2.y)) testRpm += RPM_ADJUST_STEP;
        if (buttons.pressed("x", gamepad2.x)) testRpm -= RPM_ADJUST_STEP;
        testRpm = Math.max(0, Math.min(rpmCap(), testRpm));
        if (shooterOn) robot.shooter.setTargetRpm(testRpm);
        else robot.shooter.off();
        double feed = gamepad2.right_trigger > 0.05 ? gamepad2.right_trigger * FEED_FEEDER_POWER : 0.0;
        if (feed > 0) robot.feeder.setManual(0.0, feed);
        else robot.feeder.setMode(Feeder.Mode.HOLD);

        // ---------------- drive + heading hold (gamepad 1)
        if (buttons.pressed("g1a", gamepad1.a)) { holdOn = !holdOn; hold.release(); }
        double heading = robot.localizer.getHeadingDeg();
        double base = hold.isHolding() ? hold.getTargetDeg() : heading;
        if (holdOn && buttons.pressed("g1l", gamepad1.dpad_left)) hold.setTarget(base + 90);
        if (holdOn && buttons.pressed("g1r", gamepad1.dpad_right)) hold.setTarget(base - 90);
        double driverTurn = -gamepad1.right_stick_x * TELEOP_TURN_SCALE;
        double turn = holdOn
                ? hold.update(heading, robot.localizer.getTurnRateDeg(), driverTurn, now, dt)
                : driverTurn;
        follower.setTeleOpDrive(-gamepad1.left_stick_y * 0.6, -gamepad1.left_stick_x * 0.6, turn, false);

        robot.actuate(now, dt);

        // ---------------- telemetry (numeric addData lines can be graphed in Panels)
        telemetryM.addLine("=== PIDF TUNER ===  G2: dpad select/adjust, bumpers step, A save, BACK(1s) revert");
        for (int i = 0; i < n; i++) {
            String k = PidfStore.KEYS[i];
            telemetryM.addLine(String.format("%s %-11s = %.6f   (code %.6f)", i == sel ? ">>" : "  ",
                    k, PidfStore.get(k), PidfStore.codeDefault(k)));
        }
        telemetryM.addLine(String.format("step = %.6f", step));
        telemetryM.addLine("Shooter (G2 B): " + (shooterOn ? "ON" : "off") + String.format("  test RPM (Y/X, ±%.0f) %.0f", RPM_ADJUST_STEP, testRpm));
        if (robot.shooter.getFault() != null) telemetryM.addLine("⚠ SHOOTER FAULT: " + robot.shooter.getFault());
        telemetryM.addData("shooterTargetRpm", robot.shooter.getTargetRpm());
        telemetryM.addData("shooterRpm", robot.shooter.getMeasuredRpm());
        telemetryM.addData("shooterErrorRpm", robot.shooter.getTargetRpm() - robot.shooter.getMeasuredRpm());
        telemetryM.addData("shooterPower", robot.shooter.getPower());
        telemetryM.addLine("Heading hold (G1 A): " + (holdOn ? "ON" : "off")
                + (hold.isHolding() ? "  [holding]" : "") + "   step test: G1 dpad left/right ±90°");
        telemetryM.addData("headingDeg", heading);
        telemetryM.addData("headingTargetDeg", hold.getTargetDeg());
        telemetryM.addData("headingErrorDeg", hold.getErrorDeg());
        telemetryM.addData("holdTurnCmd", hold.getOutput());
        if (!message.isEmpty()) telemetryM.addLine(message);
        telemetryM.addLine("Saved gains override RobotConstants. When done, copy them into RobotConstants and revert.");
        telemetryM.update(telemetry);
    }

    /** Keep displayed values clean (avoid 0.30000000004). */
    private static double round(double v) {
        return Math.round(v * 1e9) / 1e9;
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopMechanisms();
            robot.vision.stop();
        }
    }
}
