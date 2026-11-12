package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Turns a ShotSolver solution into motion that points the shooter at the target.
 *
 *  - {@link TurretAimer} (TURRET_FIXED = false): rotates the turret via
 *    Turret.chooseSetpoint(); returns 0 chassis turn, the driver keeps the whole drivetrain.
 *  - {@link ChassisAimer} (TURRET_FIXED = true): turns the whole robot.
 *
 * AUTO, TeleOp and the solver only talk to this interface.
 */
public interface Aimer {
    /**
     * @return chassis turn command in [-1, 1], + = counter-clockwise. 0 if this aimer doesn't
     *         use the chassis.
     */
    double update(ShotSolver.Solution s, double dt);

    /** Forget filter/derivative state (call when aiming is switched on). */
    void reset();

    String name();

    /** Build the aimer selected by RobotConstants.TURRET_FIXED. */
    static Aimer create(Turret turret) {
        return RobotConstants.TURRET_FIXED ? new ChassisAimer() : new TurretAimer(turret);
    }
}
