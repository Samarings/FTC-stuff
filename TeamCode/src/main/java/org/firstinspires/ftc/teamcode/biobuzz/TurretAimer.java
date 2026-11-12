package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Rotating turret: point the turret at the solver's target direction and leave the chassis to
 * the driver / the path. Every setpoint goes through Turret.chooseSetpoint() (±225° limits,
 * 360° flip). With nothing to aim at (no field pose and no tags seen yet) the turret holds
 * its angle.
 */
public class TurretAimer implements Aimer {
    private final Turret turret;

    public TurretAimer(Turret turret) {
        this.turret = turret;
    }

    @Override
    public double update(ShotSolver.Solution s, double dt) {
        if (s.hasAim) turret.chooseSetpoint(s.turretYawDeg);
        else turret.hold();
        return 0.0; // the chassis is not used for aiming
    }

    @Override
    public void reset() {}

    @Override
    public String name() {
        return "TURRET (±225°, flips)";
    }
}
