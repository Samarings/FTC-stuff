package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Turret rotation motor (goBILDA 312 RPM + goBILDA turret), with cable-wrap protection. There
 * is NO slip ring: the Limelight USB and the flywheel motor's cables cross the turret joint.
 *
 * Angle = UNWRAPPED degrees from the home mark (never wrapped to ±180), + = counter-clockwise
 * seen from above, 0 = camera/shot direction facing robot-forward. Travel is
 * TURRET_MIN_DEG..TURRET_MAX_DEG (±225°). The two ends overlap by 90°, so when tracking would
 * pass a limit the turret FLIPS 360° the other way to the same direction and keeps tracking.
 *
 * While RobotConstants.TURRET_FIXED is true this class never powers the motor (FLOAT, output
 * forced to 0 every loop). It still reads the encoder so the zero and calibration can be
 * checked.
 *
 * Safety rules (briefs: "Turret rotation limits", "Turret rotation range and the 360° flip"):
 *  - Home is recorded at AUTO INIT with the turret on its mark (nothing moves). TeleOp keeps
 *    AUTO's home via PoseStorage and never re-zeroes. The encoder is never reset.
 *  - All aiming goes through {@link #chooseSetpoint}: the setpoint is always inside the limits.
 *  - {@link #update} is the last line of defense in EVERY mode: zero power past the limits,
 *    slow down near them, cut power on a stall / current / encoder fault.
 *  - Manual mode stops at the limits and never flips.
 *  - Not calibrated (TURRET_TICKS_PER_DEG == 0) = never powered, because the limits can't be
 *    enforced without knowing degrees.
 */
public class Turret {
    public enum Mode { OFF, MANUAL, AIM }

    /** A flip counts as finished once the turret is this close to its new setpoint. */
    private static final double FLIP_DONE_DEG = 10.0;

    private final DcMotorEx motor; // null if not in the hardware config
    private int homeTicks;
    private int ticks;

    private Mode mode = Mode.OFF;
    private double manualPower = 0.0;
    private double setpointDeg = 0.0;
    private boolean canReach = true;
    private boolean flipping = false;
    private boolean counterRotate = false; // the setpoint follows a field direction: feed the chassis turn rate forward

    private double angleDeg = Double.NaN;
    private double rateDegS = 0.0;
    private double readSec = Double.NaN;
    private double dt = 0.02;
    private double integral = 0.0;
    private double robotYawRateDegS = 0.0;
    private double powerOut = 0.0;

    private double stallStartSec = -1.0, stallStartDeg = 0.0;
    private double currentHighSince = -1.0;
    private String fault = null;

    /**
     * @param savedHomeTicks home from PoseStorage (TeleOp after AUTO), or null to take the
     *                       current position as home (AUTO INIT, turret on its mark).
     */
    public Turret(HardwareMap hw, Integer savedHomeTicks) {
        this(find(hw), savedHomeTicks);
    }

    /** For the off-robot simulation check (TurretSimCheck): any DcMotorEx, or null. */
    Turret(DcMotorEx m, Integer savedHomeTicks) {
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

    private static DcMotorEx find(HardwareMap hw) {
        try {
            return hw.get(DcMotorEx.class, TURRET_NAME);
        } catch (Exception e) {
            return null;
        }
    }

    // ================================================================== commands

    /**
     * The ONLY way to aim. Turns a desired direction relative to the robot (any multiple of
     * 360° is the same direction) into a legal unwrapped setpoint:
     *  - while tracking, stay on the same turn as the last setpoint (no jumps);
     *  - when that would pass a limit by more than TURRET_FLIP_HYSTERESIS_DEG, flip 360° to
     *    the same direction on the other turn ({@link #isFlipping} until it gets there);
     *  - when aiming starts, take the equivalent angle closest to where the turret is now.
     *
     * @return the setpoint chosen (always inside the limits)
     */
    public double chooseSetpoint(double desiredYawDeg) {
        boolean wasAiming = mode == Mode.AIM;
        double ref = wasAiming ? setpointDeg : (Double.isNaN(angleDeg) ? 0.0 : angleDeg);
        double want = pickSetpoint(ref, desiredYawDeg);
        double sp = Range.clip(want, TURRET_MIN_DEG, TURRET_MAX_DEG);
        // Only false if the limits were narrowed below ±180 and the target is behind them.
        canReach = Math.abs(want - sp) <= TURRET_FLIP_HYSTERESIS_DEG;
        if (wasAiming && Math.abs(sp - setpointDeg) > 180.0) flipping = true;
        if (!wasAiming) integral = 0.0;
        mode = Mode.AIM;
        counterRotate = true;
        setpointDeg = sp;
        return sp;
    }

    /**
     * The angle equal to desiredYawDeg (mod 360) to go to, before clamping to the limits:
     * the one nearest refDeg, unless that is past a limit (+hysteresis) and the other way
     * around is better.
     */
    static double pickSetpoint(double refDeg, double desiredYawDeg) {
        double c = refDeg + Field.wrapDeg(desiredYawDeg - refDeg);
        double alt = c > 0 ? c - 360.0 : c + 360.0;
        double h = TURRET_FLIP_HYSTERESIS_DEG;
        return beyondLimit(c) > h && beyondLimit(alt) < beyondLimit(c) - h ? alt : c;
    }

    private static double beyondLimit(double deg) {
        return Math.max(0.0, Math.max(deg - TURRET_MAX_DEG, TURRET_MIN_DEG - deg));
    }

    /** No target to aim at: keep the last setpoint (or stay where we are if we weren't aiming). */
    public void hold() {
        counterRotate = false; // holding an angle on the robot, not a direction on the field
        if (mode == Mode.AIM) return;
        setpointDeg = Double.isNaN(angleDeg) ? 0.0 : Range.clip(angleDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
        integral = 0.0;
        mode = Mode.AIM;
    }

    /**
     * Operator stick, -1..1, + = counter-clockwise seen from above. Scaled to TURRET_MAX_POWER
     * and limited by update() like everything else. Never flips.
     */
    public void setManualPower(double p) {
        mode = Mode.MANUAL;
        flipping = false;
        manualPower = Range.clip(p, -1, 1);
    }

    public void off() {
        mode = Mode.OFF;
        flipping = false;
        manualPower = 0.0;
    }

    /** Power off immediately (end of AUTO / STOP). */
    public void stopNow() {
        off();
        powerOut = 0.0;
        if (motor != null) motor.setPower(0.0);
    }

    /** Robot turn rate (deg/s, + = CCW) for the TURRET_KV counter-rotation feedforward. */
    public void setRobotYawRate(double degPerSec) {
        robotYawRateDegS = degPerSec;
    }

    /** Try again after a STALL / OVERCURRENT (something was in the way). Keeps the zero. */
    public void clearFault() {
        fault = null;
        stallStartSec = -1.0;
        currentHighSince = -1.0;
    }

    /** Driver recovery: the turret is lined up on its home mark by hand; make this 0°. */
    public void rezeroHere() {
        homeTicks = ticks;
        angleDeg = computeAngle();
        rateDegS = 0.0;
        readSec = Double.NaN;
        clearFault();
    }

    // ================================================================== loop

    /**
     * Read the encoder. Robot.sense() calls this first so vision and the solver use this
     * loop's angle; update() calls it too (a second call with the same time does nothing).
     */
    public void read(double nowSec) {
        if (motor == null || nowSec == readSec) return;
        ticks = motor.getCurrentPosition();
        double a = computeAngle();
        dt = Double.isNaN(readSec) ? 0.02 : Math.max(0.005, nowSec - readSec);
        double rawRate = Double.isNaN(readSec) || Double.isNaN(a) || Double.isNaN(angleDeg) ? 0.0 : (a - angleDeg) / dt;
        rateDegS += 0.5 * (rawRate - rateDegS); // light filter: one encoder tick per loop is a big "speed"
        angleDeg = a;
        readSec = nowSec;
        if (isCalibrated() && !Double.isNaN(angleDeg)) {
            if (Math.abs(rawRate) > TURRET_MAX_PLAUSIBLE_DEG_S) fault = "ENCODER JUMP";
            if (angleDeg < TURRET_MIN_DEG - TURRET_FAULT_BEYOND_DEG || angleDeg > TURRET_MAX_DEG + TURRET_FAULT_BEYOND_DEG)
                fault = "ANGLE OUT OF RANGE";
        }
    }

    public void update(double nowSec) {
        if (motor == null) return;
        read(nowSec);

        double req = 0.0;
        if (mode == Mode.MANUAL) {
            req = manualPower * TURRET_MAX_POWER;
        } else if (mode == Mode.AIM && !Double.isNaN(angleDeg)) {
            double err = setpointDeg - angleDeg;
            if (Math.abs(err) < FLIP_DONE_DEG) flipping = false;
            // power = kP·e + kI·∫e − kD·(turret speed) + kF·sign(e), plus the kV counter-rotation
            if (Math.abs(err) > TURRET_DEADBAND_DEG) {
                // integrate only close to the setpoint, so a long slew or flip doesn't wind it up
                integral = Math.abs(err) < TURRET_SLOW_ZONE_DEG
                        ? Range.clip(integral + err * dt, -TURRET_INTEGRAL_CLAMP, TURRET_INTEGRAL_CLAMP) : 0.0;
                req = TURRET_KP * err + TURRET_KI * integral - TURRET_KD * rateDegS + TURRET_KF * Math.signum(err);
            }
            // robot turns +w  =>  the turret must turn -w to keep pointing the same way
            double ff = counterRotate ? -robotYawRateDegS * Math.abs(TURRET_TICKS_PER_DEG) / TURRET_MOTOR_FREE_TICKS_S : 0.0;
            req = Range.clip(req + TURRET_KV * ff, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        }

        // Limits on where the turret is AND where it will coast to, so a fast approach (manual
        // full stick) brakes early instead of sliding past the limit.
        double out = limitPower(limitPower(req, angleDeg), angleDeg + rateDegS * TURRET_LIMIT_LOOKAHEAD_S);
        if (isCalibrated()) checkStall(out, nowSec);

        // Hard gates: locked, uncalibrated, or faulted = no power at all.
        if (TURRET_FIXED || !isCalibrated() || fault != null) out = 0.0;
        powerOut = out;
        motor.setPower(out);
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

    // ================================================================== state

    public boolean isPresent() { return motor != null; }
    public static boolean isCalibrated() { return TURRET_TICKS_PER_DEG != 0.0; }
    /** Unwrapped degrees from home; NaN if missing or not calibrated. */
    public double getAngleDeg() { return angleDeg; }
    /** Angle to feed vision / the solver: 0 while locked. */
    public double getAimAngleDeg() { return TURRET_FIXED || Double.isNaN(angleDeg) ? 0.0 : angleDeg; }
    /** Turret speed, deg/s (+ = CCW). */
    public double getRateDegS() { return rateDegS; }
    public int getTicks() { return ticks; }
    public int getHomeTicks() { return homeTicks; }
    public double getSetpointDeg() { return setpointDeg; }
    public boolean canReach() { return TURRET_FIXED || canReach; }
    /** True while the turret is on its way around after a 360° flip (don't shoot). */
    public boolean isFlipping() { return !TURRET_FIXED && flipping; }
    public String getFault() { return fault; }
    public double getPower() { return powerOut; }
    public Mode getMode() { return mode; }

    public String describe() {
        if (motor == null) return "Turret: no '" + TURRET_NAME + "' motor in config" + (TURRET_FIXED ? " (OK, locked)" : "");
        String a = Double.isNaN(angleDeg) ? "uncalibrated" : String.format("%.1f°", angleDeg);
        return String.format("Turret %s: angle=%s set=%.1f° pwr=%.2f ticks=%d home=%d%s%s%s",
                TURRET_FIXED ? "LOCKED" : mode.toString(), a, setpointDeg, powerOut, ticks, homeTicks,
                isFlipping() ? "  FLIPPING" : "",
                fault != null ? "  FAULT " + fault : "", canReach() ? "" : "  OUT OF RANGE");
    }
}
