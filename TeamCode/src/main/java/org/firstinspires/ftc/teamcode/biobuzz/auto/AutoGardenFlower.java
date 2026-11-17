package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Start one tile in front of our GARDEN (heading 90°, side on the alliance wall, intake facing
 * the GARDEN) and:
 *   1. shoot the 4 pre-loads from the start spot,
 *   2. back into the GARDEN and intake its 4 POLLEN,
 *   3. drive to the far side next to the LOADING ZONE and shoot them,
 *   4. pull the 4 POLLEN out of a FLOWER (bottom opening only, G418),
 *   5. drive into the LOADING ZONE (PARK) and shoot the rest from there.
 *
 * Route option (BACK in INIT): which FLOWER (default: far wall, the one next to the LOADING
 * ZONE). Written for red; the engine converts every pose for blue.
 *
 * TeleOp hand-off: the engine saves pose, alliance, up CELL, TIP count and turret zero to
 * PoseStorage every loop, and "BIOBUZZ TeleOp" is pre-selected.
 *
 * Collection is skipped if the ball estimate says we still hold balls (G407, no sensor).
 */
@Autonomous(name = "BIOBUZZ Garden + Flower", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class AutoGardenFlower extends BiobuzzAutoBase {

    private boolean farWallFlower = true;
    /** Decided once when we reach the FLOWER leg, so its steps are skipped or run together. */
    private boolean skipFlower = false;

    { startPos = AutoPoses.Start.GARDEN; }

    @Override
    protected String routeName() { return "Garden + Flower"; }

    @Override
    protected String optionLabel() {
        return farWallFlower ? "FLOWER on the FAR wall" : "FLOWER on our ALLIANCE wall";
    }

    @Override
    protected void cycleOption() { farWallFlower = !farWallFlower; }

    private Pose[] flower() {
        return farWallFlower ? AutoPoses.flowerFarWall() : AutoPoses.flowerRedWall();
    }

    @Override
    protected List<Pose> routePosesRed() {
        List<Pose> p = new ArrayList<>(Arrays.asList(flower()));
        p.addAll(Arrays.asList(AutoPoses.gardenStartShoot(), AutoPoses.gardenBackIn(),
                AutoPoses.nearLoadingShoot(), AutoPoses.parkShoot(parkUpper)));
        return p;
    }

    /**
     * PLACEHOLDER: the FLOWER release servo isn't specified yet (config name, positions, how
     * long to wait). Does nothing and finishes at once; put the servo command here.
     */
    private Step releaseFlower() {
        return new Step("release FLOWER (servo TODO)", 0.1) {
            @Override
            boolean update(double now, double dt, ShotSolver.Solution sol) { return true; }
        };
    }

    @Override
    protected void buildRoute() {
        // 1. Pre-loads, right away. Chassis aim has to turn, and the robot starts against the
        //    wall, so it first slides clear; a rotating turret shoots from the start pose as is.
        if (TURRET_FIXED) add(new DriveStep("off the wall", AutoPoses::gardenStartShoot, true, false, 1.0));
        add(shoot(AUTO_SHOOT_STEP_TIMEOUT_S));
        add(shootRetry());

        // 2. GARDEN
        add(driveTo("into GARDEN", AutoPoses::gardenBackIn, true, AUTO_INTAKE_DRIVE_POWER).skipWhen(this::notEmpty));
        add(intakeDwell(0.5).skipWhen(this::notEmpty));

        // 3. Far side, next to the LOADING ZONE
        add(new DriveStep("to far side", AutoPoses::nearLoadingShoot, true, false, 1.0));
        add(shoot(AUTO_SHOOT_STEP_TIMEOUT_S).skipWhen(this::isEmpty));
        add(shootRetry());

        // 4. FLOWER
        add(driveTo("to FLOWER", () -> flower()[0], false, 1.0).skipWhen(() -> skipFlower = notEmpty()));
        add(driveTo("into FLOWER", () -> flower()[1], true, AUTO_INTAKE_DRIVE_POWER).skipWhen(() -> skipFlower));
        add(releaseFlower().skipWhen(() -> skipFlower));
        add(intakeDwell(AUTO_INTAKE_DWELL_S).skipWhen(() -> skipFlower));
        add(driveTo("back out", () -> flower()[0], false, 1.0).skipWhen(() -> skipFlower));

        // 5. LOADING ZONE: park facing the CELL, then shoot what is left.
        Step toZone = new DriveStep("to LOADING ZONE", () -> AutoPoses.parkShoot(parkUpper), true, false, 1.0);
        toZone.isPark = true;
        add(toZone);
        add(shoot(AUTO_SHOOT_STEP_TIMEOUT_S).skipWhen(this::isEmpty));
    }
}
