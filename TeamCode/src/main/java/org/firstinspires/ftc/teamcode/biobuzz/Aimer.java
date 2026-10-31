package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * Turns a ShotSolver solution into motion that points the shooter at the target.
 *
 *  - {@link ChassisAimer} (now, TURRET_FIXED = true): turns the whole robot.
 *  - TurretAimer (later): will call Turret.chooseSetpoint() (already written, with the
 *    cable-wrap limits) and return 0 chassis turn. Not written yet on purpose: the turret is
 *    locked for now.
 *
 * AUTO, TeleOp and the solver only talk to this interface, so switching back to the rotating
 * turret is the flag plus the new class.
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
    static Aimer create() {
        if (!RobotConstants.TURRET_FIXED) {
            throw new IllegalStateException("Rotating turret aiming (TurretAimer) is not written yet: keep TURRET_FIXED = true");
        }
        return new ChassisAimer();
    }
}
