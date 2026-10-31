package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Testing", group = "Test")
public class Testing extends OpMode {

    private DcMotorEx feeder;
    private DcMotorEx intake;
    private DcMotorEx shooter;

    // ---------------- Shooter encoder + RPM ----------------
    // Set this to encoder ticks per *flywheel* revolution.
    // If direct-drive on motor shaft, use motor ticks/rev (verify your motor).
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // ---------------- Shooter target control ----------------
    private static final int RPM_STEP = 100;
    private static final int RPM_MIN = 0;
    private static final int RPM_MAX = 7000;

    private int targetRpm = 3000;
    private boolean shooterEnabled = false;

    // ---------------- Live-tuned PIDF ----------------
    // Start values (tune in-teleop)
    private double kP = 0.014;
    private double kI = 0.0;
    private double kD = 0.00001;
    private double kF = 0.000422143; // rough starting gue ss

    // Step sizes for tuning (edit as you like)
    private static final double STEP_KP = 0.001;
    private static final double STEP_KI = 0.00000005;
    private static final double STEP_KD = 0.000001;
    private static final double STEP_KF = 0.000422143;

    // Anti-windup clamp
    private static final double INTEGRAL_CLAMP = 5000.0;

    private double integral = 0.0;
    private double lastError = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastTimeSec = 0.0;

    // ---------------- Tuning UI selection ----------------
    private enum GainSel { KP, KI, KD, KF }
    private GainSel selected = GainSel.KP;

    // Edge-detect
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastSelUp = false;
    private boolean lastSelDown = false;

    @Override
    public void init() {
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        loopTimer.reset();
        lastTimeSec = loopTimer.seconds();
    }

    @Override
    public void loop() {
        // ---------------- Feeder ----------------
        double feederPower = gamepad1.right_trigger; // 0..1
        if (gamepad1.right_bumper) feederPower = -1.0; // full reverse
        feeder.setPower(Range.clip(feederPower, -1.0, 1.0));

       // ---------------- Intake ----------------
        double intakePower = gamepad1.left_trigger; // 0..1
        if (gamepad1.left_bumper) intakePower = -1.0; // full reverse
        intake.setPower(Range.clip(intakePower, -1.0, 1.0));

        // ---------------- Target RPM adjust (dpad up/down) ----------------
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadUp && !lastDpadUp) targetRpm += RPM_STEP;
        if (dpadDown && !lastDpadDown) targetRpm -= RPM_STEP;
        targetRpm = Range.clip(targetRpm, RPM_MIN, RPM_MAX);

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;

        // ---------------- Shooter toggle (B) ----------------
        boolean b = gamepad1.b;
        if (b && !lastB) shooterEnabled = !shooterEnabled;
        lastB = b;

        // ---------------- Select which gain to edit (left stick up/down) ----------------
        // Stick up is typically negative, invert so "up" = positive
        double selStick = -gamepad1.left_stick_y;
        boolean selUp = selStick > 0.6;
        boolean selDown = selStick < -0.6;

        if (selUp && !lastSelUp) selected = nextSel(selected);
        if (selDown && !lastSelDown) selected = prevSel(selected);

        lastSelUp = selUp;
        lastSelDown = selDown;

        // ---------------- Adjust selected gain (dpad left/right) ----------------
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        // Optional: hold left stick button for 10x step
        double mult = gamepad1.left_stick_button ? 10.0 : 1.0;

        if (dpadRight && !lastDpadRight) adjustSelected(+1, mult);
        if (dpadLeft && !lastDpadLeft) adjustSelected(-1, mult);

        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;

        // ---------------- Shooter PIDF control ----------------
        double nowSec = loopTimer.seconds();
        double dt = nowSec - lastTimeSec;
        lastTimeSec = nowSec;

        if (dt <= 0.0) dt = 0.02;
        if (dt > 0.2) dt = 0.2;

        double measuredTPS = shooter.getVelocity(); // ticks/sec
        double targetTPS = (shooterEnabled && targetRpm > 0) ? rpmToTicksPerSecond(targetRpm) : 0.0;

        double error = targetTPS - measuredTPS;

        double fTerm = 0, pTerm = 0, iTerm = 0, dTerm = 0;
        double powerCmd = 0;

        if (shooterEnabled && targetRpm > 0) {
            // Terms except I (so we can do better anti-windup)
            fTerm = kF * targetTPS;
            pTerm = kP * error;
            dTerm = kD * ((error - lastError) / dt);

            double prelim = fTerm + pTerm + dTerm; // no I yet

            // Conditional integrate (anti-windup):
            // - integrate if not saturated, OR if integral would help come off the rail
            boolean wouldSaturateHigh = prelim >= 1.0;
            boolean wouldSaturateLow = prelim <= 0.0;

            boolean allowIntegrate =
                    (!wouldSaturateHigh && !wouldSaturateLow) ||
                            (wouldSaturateHigh && error < 0) || // need to reduce output
                            (wouldSaturateLow && error > 0);    // need to increase output

            if (allowIntegrate) {
                integral += error * dt;
                integral = Range.clip(integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
            }

            iTerm = kI * integral;

            powerCmd = prelim + iTerm;
            powerCmd = Range.clip(powerCmd, 0.0, 1.0);

            shooter.setPower(powerCmd);
            lastError = error;
        } else {
            shooter.setPower(0.0);
            integral = 0.0;
            lastError = 0.0;
        }

        // ---------------- Telemetry ----------------
        double measuredRpm = ticksPerSecondToRpm(measuredTPS);

        telemetry.addLine("=== Shooter PIDF Tuner ===");
        telemetry.addData("Shooter Enabled (B)", shooterEnabled);
        telemetry.addData("Target RPM (DPAD U/D)", targetRpm);
        telemetry.addData("Actual RPM", "%.0f", measuredRpm);
        telemetry.addData("Power Cmd", "%.3f", powerCmd);
        telemetry.addData("Error TPS", "%.0f", (targetTPS - measuredTPS));
        telemetry.addLine("");

        telemetry.addLine("Select gain: Left Stick Up/Down (hold L3 for 10x step)");
        telemetry.addLine("Adjust gain: DPAD Left/Right");
        telemetry.addLine(formatGainLine(selected == GainSel.KP, "kP", kP, STEP_KP));
        telemetry.addLine(formatGainLine(selected == GainSel.KI, "kI", kI, STEP_KI));
        telemetry.addLine(formatGainLine(selected == GainSel.KD, "kD", kD, STEP_KD));
        telemetry.addLine(formatGainLine(selected == GainSel.KF, "kF", kF, STEP_KF));

        telemetry.addLine("");
        telemetry.addData("Terms", "F=%.3f P=%.3f I=%.3f D=%.3f", fTerm, pTerm, iTerm, dTerm);

        telemetry.update();
    }

    private void adjustSelected(int dir, double mult) {
        double step;
        switch (selected) {
            case KP:
                step = STEP_KP * mult * dir;
                kP = clampNonNeg(kP + step);
                break;
            case KI:
                step = STEP_KI * mult * dir;
                kI = clampNonNeg(kI + step);
                break;
            case KD:
                step = STEP_KD * mult * dir;
                kD = clampNonNeg(kD + step);
                break;
            case KF:
                step = STEP_KF * mult * dir;
                kF = clampNonNeg(kF + step);
                break;
        }
    }

    private static double clampNonNeg(double v) {
        return Math.max(0.0, v);
    }

    private static GainSel nextSel(GainSel s) {
        switch (s) {
            case KP: return GainSel.KI;
            case KI: return GainSel.KD;
            case KD: return GainSel.KF;
            default: return GainSel.KP;
        }
    }

    private static GainSel prevSel(GainSel s) {
        switch (s) {
            case KP: return GainSel.KF;
            case KI: return GainSel.KP;
            case KD: return GainSel.KI;
            default: return GainSel.KD;
        }
    }

    private static String formatGainLine(boolean selected, String name, double value, double step) {
        String arrow = selected ? "-> " : "   ";
        return String.format("%s%s: %.9f   (step %.9f)", arrow, name, value, step);
    }

    private static double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * SHOOTER_TICKS_PER_REV;
    }

    private static double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
    }
}

