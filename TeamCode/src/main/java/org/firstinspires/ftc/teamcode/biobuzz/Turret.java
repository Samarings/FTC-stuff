package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Turret rotation motor, with cable-wrap protection. There is NO slip ring: the Limelight USB
 * and the flywheel motor's cables cross the turret joint, so the turret must never make a
 * full turn.
 *
 * ⚠ WIRED BUT LOCKED: while RobotConstants.TURRET_FIXED is true this class never powers the
 * motor (the motor is set to FLOAT and the output is forced to 0 every loop). It still reads
 * the encoder so the zero and the calibration can be checked.
 *
 * Safety rules (brief, "Turret rotation limits"):
 *  - Angle is UNWRAPPED degrees from the home mark, never wrapped to ±180.
 *  - Home is recorded at AUTO INIT with the turret on its mark (nothing moves). TeleOp keeps
 *    AUTO's home via PoseStorage and never re-zeroes. The encoder is never reset.
 *  - All aiming goes through {@link #chooseSetpoint}. Out-of-range targets clamp and set
 *    canReach = false (which blocks shooting).
 *  - {@link #update} is the last line of defense in EVERY mode: zero power past the limits,
 *    slow down near them, cut power on a stall / current / encoder fault.
 *  - Not calibrated (TURRET_TICKS_PER_DEG == 0) = never powered, because the limits can't be
 *    enforced without knowing degrees.
 */
public class Turret {
    public enum Mode { OFF, MANUAL, AIM }

    private final DcMotorEx motor; // null if not in the hardware config
    private int homeTicks;
    private int ticks;

    private Mode mode = Mode.OFF;
    private double manualPower = 0.0;
    private double setpointDeg = 0.0;
    private boolean canReach = true;

    private double angleDeg = Double.NaN;
    private double lastAngleDeg = Double.NaN;
    private double lastSec = -1.0;
    private double lastErr = 0.0;
    private double powerOut = 0.0;

    private double stallStartSec = -1.0, stallStartDeg = 0.0;
    private double currentHighSince = -1.0;
    private String fault = null;

    /**
     * @param savedHomeTicks home from PoseStorage (TeleOp after AUTO), or null to take the
     *                       current position as home (AUTO INIT, turret on its mark).
     */
    public Turret(HardwareMap hw, Integer savedHomeTicks) {
        DcMotorEx m = null;
        try {
            m = hw.get(DcMotorEx.class, TURRET_NAME);
        } catch (Exception e) {
            m = null;
        }
        motor = m;
        if (motor != null) {
            motor.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // never STOP_AND_RESET: keeps AUTO's zero
            motor.setZeroPowerBehavior(TURRET_FIXED ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
            ticks = motor.getCurrentPosition();
        }
        homeTicks = savedHomeTicks != null ? savedHomeTicks : ticks;
        angleDeg = computeAngle();
    }

    // ================================================================== commands

    /**
     * The ONLY way to aim. Turns a desired direction relative to the robot (-180..180) into a
     * legal unwrapped setpoint inside the soft limits.
     *
     * @param shooting true while trying to shoot: never unwind then.
     * @return the setpoint chosen (clamped to a limit if the target is out of range)
     */
    public double chooseSetpoint(double desiredYawDeg, boolean shooting) {
        mode = Mode.AIM;
        double lo = TURRET_MIN_DEG + TURRET_LIMIT_MARGIN_DEG;
        double hi = TURRET_MAX_DEG - TURRET_LIMIT_MARGIN_DEG;
        double cur = Double.isNaN(angleDeg) ? 0.0 : angleDeg;
        double yaw = Field.wrapDeg(desiredYawDeg);

        double best = Double.NaN, alt = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double c = yaw + 360.0 * k;
            if (c < lo || c > hi) continue;
            if (Double.isNaN(best) || Math.abs(c - cur) < Math.abs(best - cur)) { alt = best; best = c; }
            else if (Double.isNaN(alt) || Math.abs(c - cur) < Math.abs(alt - cur)) alt = c;
        }
        if (Double.isNaN(best)) {
            // Target is in the dead zone behind the travel range: clamp, and block shooting.
            canReach = false;
            // nearest limit, measured the short way around
            setpointDeg = Math.abs(Field.wrapDeg(yaw - lo)) < Math.abs(Field.wrapDeg(yaw - hi)) ? lo : hi;
            return setpointDeg;
        }
        canReach = true;
        // Unwind toward 0 only when not shooting and the nearer choice sits in a slow zone.
        if (!shooting && !Double.isNaN(alt) && inSlowZone(best) && Math.abs(alt) < Math.abs(best)) best = alt;
        setpointDeg = best;
        return setpointDeg;
    }

    /** Operator stick. Goes through exactly the same limits in update(). */
    public void setManualPower(double p) {
        mode = Mode.MANUAL;
        manualPower = Range.clip(p, -1, 1);
    }

    public void off() {
        mode = Mode.OFF;
        manualPower = 0.0;
    }

    /** Power off immediately (end of AUTO / STOP). */
    public void stopNow() {
        off();
        powerOut = 0.0;
        if (motor != null) motor.setPower(0.0);
    }

    /** Driver recovery: the turret is lined up on its home mark by hand; make this 0°. */
    public void rezeroHere() {
        homeTicks = ticks;
        fault = null;
        lastAngleDeg = Double.NaN;
        stallStartSec = -1.0;
        currentHighSince = -1.0;
    }

    // ================================================================== loop

    public void update(double nowSec) {
        if (motor == null) return;
        ticks = motor.getCurrentPosition();
        angleDeg = computeAngle();
        double dt = lastSec < 0 ? 0.02 : Math.max(0.005, nowSec - lastSec);
        lastSec = nowSec;

        if (isCalibrated()) checkEncoder(dt);

        double req = 0.0;
        if (mode == Mode.MANUAL) {
            req = manualPower * TURRET_MAX_POWER;
        } else if (mode == Mode.AIM && !Double.isNaN(angleDeg)) {
            double err = setpointDeg - angleDeg;
            double derr = (err - lastErr) / dt;
            lastErr = err;
            req = Math.abs(err) <= TURRET_DEADBAND_DEG ? 0.0
                    : Range.clip(TURRET_KP * err + TURRET_KD * derr, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        }

        double out = limitPower(req, angleDeg);
        if (isCalibrated()) checkStall(out, nowSec);

        // Hard gates: locked, uncalibrated, or faulted = no power at all.
        if (TURRET_FIXED || !isCalibrated() || fault != null) out = 0.0;
        powerOut = out;
        motor.setPower(out);
        lastAngleDeg = angleDeg;
    }

    /** Last line of defense: zero power into a limit, taper inside the slow zone. */
    public static double limitPower(double power, double angleDeg) {
        if (Double.isNaN(angleDeg)) return 0.0;
        if (angleDeg >= TURRET_MAX_DEG && power > 0) return 0.0;
        if (angleDeg <= TURRET_MIN_DEG && power < 0) return 0.0;
        double zone = Math.max(1e-6, TURRET_SLOW_ZONE_DEG);
        if (power > 0) {
            double room = TURRET_MAX_DEG - angleDeg;
            if (room < zone) power = Math.min(power, TURRET_MAX_POWER * room / zone);
        } else if (power < 0) {
            double room = angleDeg - TURRET_MIN_DEG;
            if (room < zone) power = Math.max(power, -TURRET_MAX_POWER * room / zone);
        }
        return power;
    }

    private void checkEncoder(double dt) {
        if (Double.isNaN(angleDeg)) return;
        if (!Double.isNaN(lastAngleDeg) && Math.abs(angleDeg - lastAngleDeg) / dt > TURRET_MAX_PLAUSIBLE_DEG_S) {
            fault = "ENCODER JUMP";
        }
        if (angleDeg < TURRET_MIN_DEG - TURRET_FAULT_BEYOND_DEG || angleDeg > TURRET_MAX_DEG + TURRET_FAULT_BEYOND_DEG) {
            fault = "ANGLE OUT OF RANGE";
        }
    }

    private void checkStall(double out, double nowSec) {
        if (Math.abs(out) > TURRET_STALL_POWER && !TURRET_FIXED) {
            if (stallStartSec < 0 || Math.abs(angleDeg - stallStartDeg) >= TURRET_STALL_MIN_DEG) {
                stallStartSec = nowSec;
                stallStartDeg = angleDeg;
            } else if (nowSec - stallStartSec > TURRET_STALL_TIME_S) {
                fault = "STALL";
            }
        } else {
            stallStartSec = -1.0;
        }
        if (TURRET_STALL_CURRENT_A > 0 && !TURRET_FIXED) {
            double amps = motor.getCurrent(CurrentUnit.AMPS);
            if (amps > TURRET_STALL_CURRENT_A) {
                if (currentHighSince < 0) currentHighSince = nowSec;
                else if (nowSec - currentHighSince > TURRET_STALL_TIME_S) fault = "OVERCURRENT";
            } else {
                currentHighSince = -1.0;
            }
        }
    }

    private double computeAngle() {
        if (motor == null || !isCalibrated()) return Double.NaN;
        return (ticks - homeTicks) / TURRET_TICKS_PER_DEG; // unwrapped, never wrapped to ±180
    }

    private static boolean inSlowZone(double deg) {
        return deg > TURRET_MAX_DEG - TURRET_SLOW_ZONE_DEG || deg < TURRET_MIN_DEG + TURRET_SLOW_ZONE_DEG;
    }

    // ================================================================== state

    public boolean isPresent() { return motor != null; }
    public static boolean isCalibrated() { return TURRET_TICKS_PER_DEG != 0.0; }
    /** Unwrapped degrees from home; NaN if missing or not calibrated. */
    public double getAngleDeg() { return angleDeg; }
    /** Angle to feed vision / the solver: 0 while locked. */
    public double getAimAngleDeg() { return TURRET_FIXED || Double.isNaN(angleDeg) ? 0.0 : angleDeg; }
    public int getTicks() { return ticks; }
    public int getHomeTicks() { return homeTicks; }
    public double getSetpointDeg() { return setpointDeg; }
    public boolean canReach() { return TURRET_FIXED || canReach; }
    public String getFault() { return fault; }
    public double getPower() { return powerOut; }
    public Mode getMode() { return mode; }

    public String describe() {
        if (motor == null) return "Turret: no '" + TURRET_NAME + "' motor in config" + (TURRET_FIXED ? " (OK, locked)" : "");
        String a = Double.isNaN(angleDeg) ? "uncalibrated" : String.format("%.1f°", angleDeg);
        return String.format("Turret %s: ticks=%d home=%d angle=%s pwr=%.2f%s%s",
                TURRET_FIXED ? "LOCKED" : mode.toString(), ticks, homeTicks, a, powerOut,
                fault != null ? "  FAULT " + fault : "", canReach() ? "" : "  OUT OF RANGE");
    }
}
