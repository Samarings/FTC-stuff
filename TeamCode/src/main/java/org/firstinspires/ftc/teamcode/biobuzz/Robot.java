package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Everything on the robot, updated in the right order. AUTO, TeleOp and the calibration
 * OpModes all use this, so the sensing/aiming logic exists exactly once.
 *
 * Per loop:
 *   robot.sense(...)        -> pose, vision, TIP state, vision correction, shot solution
 *   (OpMode decides shooter target, feeder mode, drive)
 *   robot.actuate(...)      -> shooter + feeder outputs, shot counting
 */
public class Robot {
    public Alliance alliance;
    public final Localizer localizer;
    public final Vision vision;
    public final Shooter shooter;
    public final Feeder feeder;
    public final Turret turret;
    public TipTracker tips;
    public ShotSolver solver;
    public final Aimer aimer;

    public ShotSolver.Solution solution;
    private int lastShots = 0;
    private int newShotsThisLoop = 0;

    /**
     * @param autoInit true for AUTO-type OpModes: the turret home is recorded here (turret
     *                 must be on its mark). false for TeleOp-type OpModes: the turret keeps
     *                 AUTO's home if AUTO just ran, otherwise it assumes it's on its mark.
     */
    public Robot(HardwareMap hw, Alliance alliance, CellSide initialUpCell, boolean autoInit) {
        this.alliance = alliance;
        localizer = new Localizer(hw);
        vision = new Vision(hw, alliance);
        shooter = new Shooter(hw);
        feeder = new Feeder(hw);
        Integer savedHome = (!autoInit && PoseStorage.isFresh(600.0)) ? PoseStorage.turretHomeTicks : null;
        turret = new Turret(hw, savedHome);
        tips = new TipTracker(alliance, initialUpCell);
        solver = new ShotSolver(alliance);
        aimer = Aimer.create();
        lastShots = shooter.getShotsDetected();
    }

    /** Re-select alliance / starting up CELL (INIT menu). Resets TIP tracking. */
    public void configure(Alliance a, CellSide upCell) {
        alliance = a;
        vision.setAlliance(a);
        tips = new TipTracker(a, upCell);
        solver = new ShotSolver(a);
    }

    /** Sense + solve. Call once per loop before deciding outputs. */
    public ShotSolver.Solution sense(double nowSec, BallType ball, boolean periodAllowsShots,
                                     double rpmTrim, boolean allowVisionApply) {
        localizer.update(nowSec);
        vision.update(localizer.getHeadingDeg(), turret.getAimAngleDeg() /* 0 while locked */, nowSec);
        tips.update(vision, nowSec);
        localizer.offerVision(vision.getPoseFix(), tips, allowVisionApply, nowSec);
        int balls = feeder.isBallCountKnown() ? feeder.getBallEstimate() : -1;
        solution = solver.solve(localizer, vision, tips, shooter, ball, balls, periodAllowsShots, rpmTrim, nowSec);
        if (!RobotConstants.TURRET_FIXED && solution.canShoot) {
            if (turret.getFault() != null) { solution.canShoot = false; solution.whyNot = "TURRET FAULT " + turret.getFault(); }
            else if (!turret.canReach()) { solution.canShoot = false; solution.whyNot = "TARGET OUT OF TURRET RANGE"; }
        }
        return solution;
    }

    /** Push shooter + feeder outputs and count launched balls. */
    public void actuate(double nowSec, double dt) {
        shooter.setFeeding(feeder.isFeeding());
        shooter.update(nowSec, dt);
        int shots = shooter.getShotsDetected();
        newShotsThisLoop = shots - lastShots;
        lastShots = shots;
        if (newShotsThisLoop > 0) {
            feeder.onShotsLaunched(newShotsThisLoop);
            tips.onShotsIntoCell(newShotsThisLoop);
        }
        feeder.update(nowSec);
        turret.update(nowSec); // always 0 power while TURRET_FIXED
    }

    public int newShotsThisLoop() { return newShotsThisLoop; }

    /** All mechanisms off (end of AUTO). Drive is the OpMode's job. */
    public void stopMechanisms() {
        shooter.stopNow();
        feeder.stopNow();
        turret.stopNow();
    }

    /** Save state for TeleOp. */
    public void savePose() {
        PoseStorage.save(localizer.getPose(), alliance, tips.getUpCell(), tips.getTipCount());
        if (turret.isPresent()) PoseStorage.turretHomeTicks = turret.getHomeTicks();
    }
}
