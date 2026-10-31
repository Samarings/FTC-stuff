package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp heading hold: a PIDF on the Pinpoint heading.
 *
 * While the driver turns (stick outside the deadband) the driver's turn passes straight
 * through. When the stick is released, the robot is allowed to stop rotating, then the
 * current heading is locked and held against bumps and drift.
 *
 *   error = target - heading (deg, wrapped, + = need to turn counter-clockwise)
 *   turn  = KP*e + KI*∫e + KD*de/dt + KF*sign(e)     (0 inside the deadband)
 *
 * KF is a static-friction kick so small errors still move the robot.
 */
public class HeadingHold {
    private boolean holding = false;
    private double targetDeg = 0.0;
    private double releasedAtSec = -1.0;
    private double integral = 0.0, lastErr = 0.0;
    private boolean firstLoop = true;
    private double lastErrorDeg = 0.0, lastOutput = 0.0;

    /**
     * @param headingDeg  current Pinpoint heading (deg)
     * @param turnRateDeg current turn rate (deg/s, absolute)
     * @param driverTurn  driver's turn command [-1, 1] (+ = CCW)
     * @return the turn command to send to the drive
     */
    public double update(double headingDeg, double turnRateDeg, double driverTurn, double nowSec, double dt) {
        if (!HEADING_HOLD_ENABLED || Math.abs(driverTurn) > HOLD_STICK_DEADBAND) {
            release();
            lastOutput = driverTurn;
            return driverTurn;
        }
        if (!holding) {
            if (releasedAtSec < 0) releasedAtSec = nowSec;
            boolean settled = turnRateDeg < HOLD_CAPTURE_RATE_DEG_S || nowSec - releasedAtSec > HOLD_CAPTURE_DELAY_S;
            if (!settled) {
                lastOutput = 0.0;
                return 0.0; // let the robot stop turning before locking the heading
            }
            setTarget(headingDeg);
        }
        double err = Field.wrapDeg(targetDeg - headingDeg);
        double derr = firstLoop ? 0.0 : (err - lastErr) / Math.max(1e-3, dt);
        firstLoop = false;
        lastErr = err;
        lastErrorDeg = err;

        double out;
        if (Math.abs(err) <= HOLD_DEADBAND_DEG) {
            out = 0.0;
        } else {
            integral = Range.clip(integral + err * dt, -HOLD_INTEGRAL_CLAMP, HOLD_INTEGRAL_CLAMP);
            out = HOLD_KP * err + HOLD_KI * integral + HOLD_KD * derr + HOLD_KF * Math.signum(err);
            out = Range.clip(out, -HOLD_MAX_POWER, HOLD_MAX_POWER);
        }
        lastOutput = out;
        return out;
    }

    /** Hold this heading now (e.g. after a pose reset, or for a step test). */
    public void setTarget(double headingDeg) {
        targetDeg = Field.wrapDeg(headingDeg);
        holding = true;
        integral = 0.0;
        firstLoop = true;
    }

    /** Stop holding (driver turning, auto-aim active, or hold switched off). */
    public void release() {
        holding = false;
        releasedAtSec = -1.0;
        integral = 0.0;
        firstLoop = true;
    }

    public boolean isHolding() { return holding; }
    public double getTargetDeg() { return targetDeg; }
    public double getErrorDeg() { return holding ? lastErrorDeg : 0.0; }
    public double getOutput() { return lastOutput; }
}
