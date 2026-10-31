package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.util.Range;

/**
 * Fixed turret: aim by rotating the chassis (DECODE-style). Only takes over ROTATION; the
 * driver keeps field-centric translation. PD on the solver's aim error, with a deadband and a
 * lower power cap close to the target. Gains must be retuned for the new robot.
 */
public class ChassisAimer implements Aimer {
    private double lastErr = 0.0;
    private boolean first = true;

    @Override
    public double update(ShotSolver.Solution s, double dt) {
        double err = s.aimErrDeg; // + = need to turn CCW
        double derr = first ? 0.0 : (err - lastErr) / Math.max(1e-3, dt);
        first = false;
        lastErr = err;
        if (Math.abs(err) <= AIM_DEADBAND_DEG) return 0.0;
        double max = Math.abs(err) <= AIM_FINE_ZONE_DEG ? AIM_FINE_MAX_POWER : AIM_MAX_POWER;
        return Range.clip(AIM_KP * err + AIM_KD * derr, -max, max);
    }

    @Override
    public void reset() {
        first = true;
        lastErr = 0.0;
    }

    @Override
    public String name() {
        return "CHASSIS (turret fixed)";
    }
}
