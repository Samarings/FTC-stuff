package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Fixed-hood flywheel (one goBILDA 5203). Custom PIDF in power units, reused from DECODE.
 *
 * Every commanded RPM is clamped to [0, RobotConstants.rpmCap()] (3500 by default, never
 * above 6000). Call {@link #update(double, double)} exactly once per loop.
 *
 * Also counts launched balls by watching for an RPM dip while the feeder runs (no sensor).
 */
public class Shooter {
    public enum Mode { OFF, IDLE, ACTIVE }

    private final DcMotorEx motor;

    private Mode mode = Mode.OFF;
    private double targetRpm = 0.0;

    private double integral = 0.0, lastError = 0.0;
    private double powerCmd = 0.0;
    private double kickEndSec = -1.0;
    private boolean wasActive = false;

    private double measuredRpm = 0.0;
    private int readyLoops = 0;

    // shot detection
    private String fault = null;
    private double faultSuspectSince = -1.0;
    private boolean feeding = false;
    private boolean dipArmed = true;
    private int shotsDetected = 0;

    public Shooter(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, SHOOTER_NAME);
        motor.setDirection(SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);
    }

    /** Closed-loop spin to an RPM (clamped to the cap). */
    public void setTargetRpm(double rpm) {
        mode = Mode.ACTIVE;
        targetRpm = clampRpm(rpm);
    }

    /** Open-loop idle (TeleOp). Never counts as "ready". */
    public void idle() {
        mode = Mode.IDLE;
        fault = null; // leaving ACTIVE clears a fault (fix the cause first!)
        faultSuspectSince = -1.0;
    }

    /** Turning the shooter off (or to idle) clears a fault (fix the cause first!). */
    public void off() {
        mode = Mode.OFF;
        fault = null;
        faultSuspectSince = -1.0;
    }

    /** Tell the shot detector whether the feeder is pushing balls right now. */
    public void setFeeding(boolean feeding) {
        this.feeding = feeding;
    }

    public static double clampRpm(double rpm) {
        return Range.clip(rpm, 0.0, rpmCap());
    }

    public void update(double nowSec, double dt) {
        measuredRpm = motor.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV;
        if (dt <= 0) dt = 0.02;

        boolean active = mode == Mode.ACTIVE && targetRpm > 1.0 && fault == null;

        if (active) {
            if (!wasActive && measuredRpm < targetRpm * 0.5) {
                kickEndSec = nowSec + SHOOTER_KICK_SEC; // spin-up kick from (near) stopped
                resetPid();
            }
            if (nowSec < kickEndSec) {
                powerCmd = 1.0;
            } else {
                double targetTps = targetRpm / 60.0 * SHOOTER_TICKS_PER_REV;
                double error = targetTps - motor.getVelocity();
                double prelim = SHOOTER_KF * targetTps + SHOOTER_KP * error
                        + SHOOTER_KD * (error - lastError) / dt;
                boolean satHigh = prelim >= 1.0, satLow = prelim <= 0.0;
                if ((!satHigh && !satLow) || (satHigh && error < 0) || (satLow && error > 0)) {
                    integral = Range.clip(integral + error * dt, -SHOOTER_INTEGRAL_CLAMP, SHOOTER_INTEGRAL_CLAMP);
                }
                powerCmd = Range.clip(prelim + SHOOTER_KI * integral, 0.0, 1.0);
                lastError = error;
            }
        } else {
            resetPid();
            kickEndSec = -1.0;
            double goal = 0.0;
            if (mode == Mode.IDLE) {
                double idleTps = clampRpm(IDLE_RPM) / 60.0 * SHOOTER_TICKS_PER_REV;
                goal = Range.clip(SHOOTER_KF * idleTps, 0.0, 1.0);
            }
            double step = SHOOTER_RAMP_PER_SEC * dt;
            double d = goal - powerCmd;
            powerCmd = Math.abs(d) <= step ? goal : powerCmd + Math.signum(d) * step;
        }
        // Encoder / direction sanity: hard power but the encoder says ~0 or backwards means an
        // unplugged encoder or a wrong SHOOTER_REVERSED. The PID would otherwise run the motor
        // at full power (~6000 RPM, past RPM_CAP). Cut power and report.
        if (active && powerCmd >= 0.6 && measuredRpm < 150.0) {
            if (faultSuspectSince < 0) faultSuspectSince = nowSec;
            else if (nowSec - faultSuspectSince > SHOOTER_FAULT_TIME_S) {
                fault = measuredRpm < -150.0 ? "RPM NEGATIVE: flip SHOOTER_REVERSED"
                        : "NO RPM: check shooter encoder cable";
                powerCmd = 0.0;
                resetPid();
            }
        } else {
            faultSuspectSince = -1.0;
        }
        if (fault != null) powerCmd = 0.0;

        wasActive = active;
        motor.setPower(powerCmd);

        // ready band
        if (active && Math.abs(measuredRpm - targetRpm) <= SHOOTER_READY_TOL_RPM) readyLoops++;
        else readyLoops = 0;

        // shot detection: a dip below target while feeding = one ball launched
        if (active && feeding && dipArmed && measuredRpm < targetRpm - SHOT_DIP_RPM) {
            shotsDetected++;
            dipArmed = false;
        }
        if (!dipArmed && (!active || measuredRpm >= targetRpm - SHOT_REARM_RPM)) {
            dipArmed = true;
        }
    }

    private void resetPid() {
        integral = 0.0;
        lastError = 0.0;
    }

    /** Stop immediately (end of AUTO, G403/G404). */
    public void stopNow() {
        mode = Mode.OFF;
        powerCmd = 0.0;
        resetPid();
        motor.setPower(0.0);
    }

    public boolean isReady() {
        return mode == Mode.ACTIVE && fault == null && targetRpm > 1.0 && readyLoops >= SHOOTER_READY_LOOPS;
    }

    public Mode getMode() { return mode; }
    /** Non-null if the shooter cut itself off (encoder/direction problem). Cleared by off(). */
    public String getFault() { return fault; }
    public double getTargetRpm() { return mode == Mode.ACTIVE ? targetRpm : 0.0; }
    public double getMeasuredRpm() { return measuredRpm; }
    public double getPower() { return powerCmd; }
    /** Total balls detected leaving the shooter since construction. */
    public int getShotsDetected() { return shotsDetected; }
}
