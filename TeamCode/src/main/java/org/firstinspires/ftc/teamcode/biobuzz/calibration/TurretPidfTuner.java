package org.firstinspires.ftc.teamcode.biobuzz.calibration;

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
import org.firstinspires.ftc.teamcode.biobuzz.Feeder;
import org.firstinspires.ftc.teamcode.biobuzz.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConstants;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

/**
 * Tune the turret PIDF and check the AprilTag lock. Needs TURRET_FIXED = false and
 * TURRET_TICKS_PER_DEG set (run Turret Calibration first). Turret ON ITS HOME MARK at INIT.
 *
 * Gains are edited live in Panels (Configurables -> RobotConstants): TURRET_KP, KI, KD, KF,
 * KV, TURRET_DEADBAND_DEG, TURRET_MAX_POWER. When done, type the final numbers into
 * RobotConstants.java (Panels edits are lost when the app restarts).
 *
 * GAMEPAD 2 (turret)
 *   X                  MANUAL: left stick X turns the turret (right = clockwise). Stops at ±225°.
 *   A                  STEP test: every press jumps to the other angle of the pair
 *   dpad left / right  step pair: 0° <-> 90°, or +120° <-> -120° (goes the long way round
 *                      through 0° every time: this is the 360° flip logic)
 *   Y                  TRACK: lock onto our up CELL's tags (the match auto-aim). Push / drive /
 *                      spin the robot: the turret should stay on the cluster and flip at ±225°.
 *   B                  turret off (also clears a STALL fault)
 * GAMEPAD 1: drive (so you can test TRACK while moving). X / B in INIT = alliance.
 *
 * Order: KP up until a step is quick with a little overshoot, KD until the overshoot is gone,
 * KF only if small errors don't move the turret, KI usually 0. Then TRACK: spin the robot
 * and adjust KV (starts at 1.0 = the motor's free speed) until the error while spinning stays
 * small. If the error gets WORSE with KV, the turret direction or ticks/deg is wrong.
 */
@TeleOp(name = "Turret PIDF Tuner", group = "Calibration")
public class TurretPidfTuner extends OpMode {
    private enum Mode { OFF, MANUAL, STEP, TRACK }

    private static final double[][] PAIRS = {{0, 90}, {120, -120}};
    private static final double SETTLE_BAND_DEG = 2.0;

    private Robot robot;
    private Follower follower;
    private TelemetryManager telemetryM;
    private final Buttons buttons = new Buttons();
    private Alliance alliance = Alliance.RED;

    private Mode mode = Mode.OFF;
    private int pair = 0, side = 0;
    private double stepStartSec, stepDir, overshootDeg, lastOutsideSec;
    private double lastSec;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Robot(hardwareMap, alliance, alliance.startUpCell(), false);
        follower = robot.localizer.follower();
        robot.localizer.resetImuAndSetPose(new Pose(0, 0, 0));
        robot.fieldPose = false; // aim from the tags only, like TeleOp
    }

    @Override
    public void init_loop() {
        if (buttons.pressed("x", gamepad1.x)) { alliance = Alliance.BLUE; robot.configure(alliance, alliance.startUpCell()); }
        if (buttons.pressed("b", gamepad1.b)) { alliance = Alliance.RED; robot.configure(alliance, alliance.startUpCell()); }
        telemetry.addLine("Turret PIDF Tuner. Alliance (G1 X/B): " + alliance);
        telemetry.addLine("Turret ON ITS HOME MARK (camera facing robot-forward), robot still.");
        if (TURRET_FIXED) telemetry.addLine("⚠ TURRET_FIXED is true: the turret will NOT move. Set it to false.");
        if (!Turret.isCalibrated()) telemetry.addLine("⚠ TURRET_TICKS_PER_DEG is 0: run Turret Calibration first.");
        telemetry.addLine("Nothing moves until START.");
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

        ShotSolver.Solution sol = robot.sense(now, BallType.POLLEN, true, 0.0, false);
        follower.setTeleOpDrive(-gamepad1.left_stick_y * 0.6, -gamepad1.left_stick_x * 0.6,
                -gamepad1.right_stick_x * TELEOP_TURN_SCALE, false);

        Turret turret = robot.turret;
        if (buttons.pressed("g2x", gamepad2.x)) mode = Mode.MANUAL;
        if (buttons.pressed("g2y", gamepad2.y)) mode = Mode.TRACK;
        if (buttons.pressed("g2b", gamepad2.b)) { mode = Mode.OFF; turret.clearFault(); }
        boolean newStep = false;
        if (buttons.pressed("g2l", gamepad2.dpad_left)) { pair = 0; newStep = mode == Mode.STEP; }
        if (buttons.pressed("g2r", gamepad2.dpad_right)) { pair = 1; newStep = mode == Mode.STEP; }
        if (buttons.pressed("g2a", gamepad2.a)) {
            if (mode == Mode.STEP) side = 1 - side;
            mode = Mode.STEP;
            newStep = true;
        }

        switch (mode) {
            case MANUAL:
                turret.setManualPower(Math.abs(gamepad2.left_stick_x) > 0.05 ? -gamepad2.left_stick_x : 0.0);
                break;
            case STEP:
                turret.chooseSetpoint(PAIRS[pair][side]);
                break;
            case TRACK:
                robot.aimer.update(sol, dt);
                break;
            default:
                turret.off();
        }
        double angle = turret.getAngleDeg(), set = turret.getSetpointDeg(), err = set - angle;
        if (newStep) {
            stepStartSec = now;
            lastOutsideSec = now;
            stepDir = Math.signum(err);
            overshootDeg = 0.0;
        }
        if (mode == Mode.STEP) {
            overshootDeg = Math.max(overshootDeg, -err * stepDir); // how far past the setpoint it went
            if (Math.abs(err) > SETTLE_BAND_DEG) lastOutsideSec = now;
        }

        robot.shooter.off();
        robot.feeder.setMode(Feeder.Mode.HOLD);
        robot.actuate(now, dt);

        // ---------------- telemetry (numeric addData lines can be graphed in Panels)
        telemetryM.addLine("=== TURRET PIDF TUNER ===  G2: X manual, A step, dpad L/R pair, Y track, B off");
        telemetryM.addLine("Mode " + mode + (mode == Mode.STEP ? String.format("  target %.0f° (pair %.0f° / %.0f°)",
                PAIRS[pair][side], PAIRS[pair][0], PAIRS[pair][1]) : ""));
        if (TURRET_FIXED) telemetryM.addLine("⚠ TURRET_FIXED is true: the turret will NOT move.");
        if (!Turret.isCalibrated()) telemetryM.addLine("⚠ TURRET_TICKS_PER_DEG is 0: run Turret Calibration first.");
        telemetryM.addLine(turret.describe());
        telemetryM.addData("turretSetpointDeg", set);
        telemetryM.addData("turretAngleDeg", angle);
        telemetryM.addData("turretErrorDeg", err);
        telemetryM.addData("turretPower", turret.getPower());
        if (mode == Mode.STEP) telemetryM.addLine(String.format("Step: overshoot %.1f°   settled (within %.0f°) after %.2f s%s",
                overshootDeg, SETTLE_BAND_DEG, lastOutsideSec - stepStartSec,
                Math.abs(err) > SETTLE_BAND_DEG ? "  (still moving)" : ""));
        if (mode == Mode.TRACK) {
            telemetryM.addLine(String.format("Track: %s  aim error %.1f° (tolerance %.1f°)  dist %.0f in  %s",
                    !sol.hasAim ? "NO TARGET YET (point the camera at our tags)" : sol.visionAim ? "LOCKED on tags" : "tags lost: odometry",
                    sol.aimErrDeg, sol.toleranceDeg, sol.distanceIn, sol.aimed ? "AIMED" : ""));
            telemetryM.addData("aimErrorDeg", sol.aimErrDeg);
            telemetryM.addLine("Our " + sol.upCell + " cluster: " + robot.vision.ourCluster(sol.upCell).centerMethod()
                    + "   visible IDs: " + robot.vision.getVisibleIds());
        }
        telemetryM.addLine(String.format("KP %.4f  KI %.4f  KD %.5f  KF %.3f  KV %.5f  deadband %.2f°  max power %.2f",
                TURRET_KP, TURRET_KI, TURRET_KD, TURRET_KF, TURRET_KV, TURRET_DEADBAND_DEG, TURRET_MAX_POWER));
        for (String w : RobotConstants.placeholderWarnings()) if (w.contains("TURRET") || w.contains("CAMERA")) telemetryM.addLine("⚠ " + w);
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopMechanisms();
            robot.vision.stop();
        }
    }
}
