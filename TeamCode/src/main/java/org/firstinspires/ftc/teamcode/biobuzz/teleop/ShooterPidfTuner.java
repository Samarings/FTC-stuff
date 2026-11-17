package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter, feeder, and intake test / PIDF tuner for Cheetahs BIOBUZZ.
 * Matches the custom tick/second PIDF calculation in MainTeleOp.
 * No turret or Pedro follower is initialized. All controls use Gamepad 1.
 * Power is forced to zero during INIT, when disabled, and in stop().
 *
 * G1 A: enable/disable flywheel
 * G1 D-pad up/down: target RPM +/-100
 * G1 LB/RB: select previous/next gain (P, I, D, F)
 * G1 X/Y: decrease/increase selected gain
 * G1 D-pad left/right: decrease/increase tuning step (x0.1 / x10)
 * G1 B: reset PID integral (without changing gains)
 *
 * G1 RT: feeder forward (proportional). G1 LT: intake forward (fixed power).
 * G1 right-stick button (hold): reverse BOTH for eject. Release to stop.
 * G1 BACK: emergency stop latch. G1 START clears latch with triggers released.
 * Tune WITHOUT balls first, then feed one ball at a time when the flywheel is stable.
 */
@Configurable
@TeleOp(name = "BIOBUZZ Shooter PIDF Tuner", group = "BIOBUZZ Calibration")
public class ShooterPidfTuner extends OpMode {
    public static String SHOOTER_NAME = "shooter";
    public static String FEEDER_NAME = "feeder";
    public static String INTAKE_NAME = "intake";
    // Same directions as the current simplified MainTeleOp.
    public static boolean FEEDER_REVERSED = true;
    public static boolean INTAKE_REVERSED = true;
    public static double FEEDER_MAX_POWER = 1.0;
    public static double INTAKE_MAX_POWER = 1.0;
    public static double EJECT_POWER = 0.5;
    // Match your MainTeleOp exactly. This is motor encoder ticks/revolution,
    // NOT turret encoder ticks/revolution.
    public static double TICKS_PER_REV = 28.0;
    public static boolean SHOOTER_REVERSED = false;
    public static double MAX_RPM = 3500.0;
    public static double TARGET_RPM = 2300.0;
    public static double RPM_INCREMENT = 100.0;

    // Same units as MainTeleOp: errors are in ENCODER TICKS/SECOND.
    // Copy your successful tuned values back into MainTeleOp.SHOOTER_P/I/D/F.
    public static double KP = 0.015;
    public static double KI = 0.0;
    public static double KD = 0.0001;
    public static double KF = 0.000422143;

    // Keep these aligned with MainTeleOp while comparing behavior.
    public static double KICK_SEC = 0.15;
    public static double OUTPUT_RAMP_PER_SEC = 3.0;
    public static double INTEGRAL_LIMIT = 5000.0;
    public static double MIN_RPM_ENCODER_CHECK = 150.0;

    private static final String[] NAMES = {"P", "I", "D", "F"};
    private static final double[] BASE_STEPS = {0.0001, 0.00001, 0.00001, 0.00001};
    private DcMotorEx motor;
    private DcMotor feeder, intake;
    private boolean emergencyStopped = false;
    private boolean lastBack = false, lastStart = false;
    private double feederPower = 0.0, intakePower = 0.0;
    private boolean enabled = false;
    private boolean pa, pb, px, py, plb, prb, pup, pdown, pleft, pright;
    private int selected = 0;
    private double stepScale = 1.0;
    private double lastTime, startedAt = -1.0, integral = 0.0, lastError = 0.0;
    private double output = 0.0, measuredRpm = 0.0, errorRpm = 0.0;
    private String status = "OFF";

    @Override public void init() {
        motor = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        motor.setDirection(SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0.0);
        feeder = hardwareMap.get(DcMotor.class, FEEDER_NAME);
        intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        feeder.setDirection(FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setPower(0.0);
        intake.setPower(0.0);
        telemetry.addLine("Only shooter, feeder and intake are controlled.");
        telemetry.addLine("Verify TICKS_PER_REV and motor direction BEFORE enabling.");
        telemetry.addLine("Press START, then G1 A to spin. Keep balls removed.");
        telemetry.update();
    }

    @Override public void start() { lastTime = getRuntime(); }

    private static boolean edge(boolean now, boolean before) { return now && !before; }

    @Override public void loop() {
        double now = getRuntime();
        double dt = Range.clip(now - lastTime, 0.005, 0.1);
        lastTime = now;

        // G1 BACK latches an emergency stop; no motor auto-restarts after reset.
        if (edge(gamepad1.back, lastBack)) {
            emergencyStopped = true;
            enabled = false;
            resetPid();
            motor.setPower(0.0);
        }
        if (edge(gamepad1.start, lastStart)
                && gamepad1.right_trigger < 0.05
                && gamepad1.left_trigger < 0.05
                && !gamepad1.right_stick_button) {
            emergencyStopped = false;
        }
        lastBack = gamepad1.back;
        lastStart = gamepad1.start;

        if (!emergencyStopped && edge(gamepad1.a, pa)) {
            enabled = !enabled;
            resetPid();
            if (!enabled) { output = 0.0; motor.setPower(0.0); }
        }
        if (edge(gamepad1.b, pb)) resetPid();
        if (edge(gamepad1.dpad_up, pup)) TARGET_RPM = Range.clip(TARGET_RPM + RPM_INCREMENT, 0, MAX_RPM);
        if (edge(gamepad1.dpad_down, pdown)) TARGET_RPM = Range.clip(TARGET_RPM - RPM_INCREMENT, 0, MAX_RPM);
        if (edge(gamepad1.left_bumper, plb)) selected = (selected + 3) % 4;
        if (edge(gamepad1.right_bumper, prb)) selected = (selected + 1) % 4;
        if (edge(gamepad1.dpad_left, pleft)) stepScale = Math.max(0.01, stepScale / 10.0);
        if (edge(gamepad1.dpad_right, pright)) stepScale = Math.min(100.0, stepScale * 10.0);
        double change = (edge(gamepad1.y, py) ? 1.0 : 0.0) - (edge(gamepad1.x, px) ? 1.0 : 0.0);
        if (change != 0.0) {
            double next = Math.max(0.0, getGain(selected) + change * BASE_STEPS[selected] * stepScale);
            setGain(selected, next);
            resetPid();
        }
        pa=gamepad1.a; pb=gamepad1.b; px=gamepad1.x; py=gamepad1.y;
        plb=gamepad1.left_bumper; prb=gamepad1.right_bumper;
        pup=gamepad1.dpad_up; pdown=gamepad1.dpad_down;
        pleft=gamepad1.dpad_left; pright=gamepad1.dpad_right;

        double velocityTicksS = motor.getVelocity();
        measuredRpm = TICKS_PER_REV > 0.0 ? velocityTicksS * 60.0 / TICKS_PER_REV : Double.NaN;
        errorRpm = TARGET_RPM - measuredRpm;

        if (emergencyStopped || !enabled || TARGET_RPM <= 0.0 || !Double.isFinite(measuredRpm) || TICKS_PER_REV <= 0.0) {
            output = 0.0;
            motor.setPower(0.0);
            startedAt = -1.0;
            integral = 0.0;
            lastError = 0.0;
            status = emergencyStopped ? "EMERGENCY STOP" : enabled ? "INVALID INPUT / ZERO TARGET" : "OFF";
        } else {
            if (startedAt < 0.0) {
                startedAt = now;
                integral = 0.0;
                lastError = 0.0;
            }
            double targetTicksS = TARGET_RPM * TICKS_PER_REV / 60.0;
            double errorTicksS = targetTicksS - velocityTicksS;
            if (now - startedAt < KICK_SEC) {
                output = 1.0;
                status = "STARTUP KICK";
            } else {
                // Mirrors MainTeleOp: custom tick/s PIDF, NOT REV's built-in PIDF.
                integral = Range.clip(integral + errorTicksS * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
                double p = KF * targetTicksS + KP * errorTicksS
                        + KD * (errorTicksS - lastError) / dt + KI * integral;
                double desiredPower = Range.clip(p, 0.0, 1.0);
                // Original MainTeleOp does not ramp while enabled, so preserve behavior.
                output = desiredPower;
                status = "PIDF";
                if (output > 0.6 && measuredRpm < MIN_RPM_ENCODER_CHECK) {
                    output = 0.0;
                    status = "ENCODER / STALL CHECK";
                }
            }
            lastError = errorTicksS;
            motor.setPower(output);
        }

        if (emergencyStopped) {
            feederPower = 0.0;
            intakePower = 0.0;
            motor.setPower(0.0);
            output = 0.0;
        } else if (gamepad1.right_stick_button) {
            feederPower = -Math.abs(Range.clip(EJECT_POWER, 0.0, 1.0));
            intakePower = feederPower;
        } else {
            feederPower = gamepad1.right_trigger > 0.05
                    ? Range.clip(gamepad1.right_trigger * FEEDER_MAX_POWER, 0.0, 1.0) : 0.0;
            intakePower = gamepad1.left_trigger > 0.05
                    ? Range.clip(INTAKE_MAX_POWER, 0.0, 1.0) : 0.0;
        }
        feeder.setPower(feederPower);
        intake.setPower(intakePower);

        telemetry.addData("E-STOP", emergencyStopped ? "LATCHED - G1 START to clear" : "clear");
        telemetry.addData("Feeder / Intake", "%.2f / %.2f", feederPower, intakePower);
        telemetry.addData("Shooter", "%s (%s)", enabled ? "ON" : "OFF", status);
        telemetry.addData("Target / Actual", "%.0f / %.0f RPM", TARGET_RPM, measuredRpm);
        telemetry.addData("RPM Error", "%+.1f", errorRpm);
        telemetry.addData("Motor power", "%.3f", output);
        telemetry.addData("Velocity", "%.1f ticks/s", velocityTicksS);
        telemetry.addData("PIDF", "P=%.8f I=%.8f D=%.8f F=%.8f", KP, KI, KD, KF);
        telemetry.addData("Selected", "%s: step %.8f", NAMES[selected], BASE_STEPS[selected]*stepScale);
        telemetry.addLine("A toggle | Up/Down RPM | LB/RB gain | X/Y -/+ | Left/Right step | B reset I");
        telemetry.addLine("G1 RT feed | LT intake | RS click eject | BACK E-STOP | START reset");
        telemetry.addLine("Copy tuned gains to MainTeleOp; they are not saved automatically.");
        telemetry.update();
    }

    private void resetPid() { integral = 0.0; lastError = 0.0; startedAt = -1.0; }
    private static double getGain(int which) {
        switch(which) { case 0: return KP; case 1: return KI; case 2: return KD; default: return KF; }
    }
    private static void setGain(int which, double value) {
        switch(which) { case 0: KP=value; break; case 1: KI=value; break; case 2: KD=value; break; default: KF=value; }
    }
    @Override public void stop() {
        if (motor != null) motor.setPower(0.0);
        if (feeder != null) feeder.setPower(0.0);
        if (intake != null) intake.setPower(0.0);
    }
}
