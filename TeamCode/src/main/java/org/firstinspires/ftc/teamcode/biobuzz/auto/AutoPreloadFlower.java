package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Shoot the pre-loads, pull the 4 POLLEN out of one of our FLOWERS' BOTTOM retrieval opening
 * (G418: POLLEN only, bottom only), shoot them, park.
 *
 * Route option (BACK in INIT): which FLOWER. Coordinate with the partner in queue: one robot
 * takes the far-wall FLOWER, the other the red-wall FLOWER / GARDEN.
 *
 * Collection is skipped if the ball estimate says we still hold balls (G407, no sensor).
 */
@Autonomous(name = "BIOBUZZ Preload + Flower", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class AutoPreloadFlower extends BiobuzzAutoBase {

    private boolean farWallFlower = false;

    @Override
    protected String routeName() { return "Preload + Flower"; }

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
        return new ArrayList<>(Arrays.asList(flower()));
    }

    @Override
    protected void buildRoute() {
        add(driveToShootSpot());
        add(shoot(AUTO_SHOOT_STEP_TIMEOUT_S));
        add(shootRetry());

        add(driveTo("to FLOWER", () -> flower()[0], false, 1.0).skipWhen(this::notEmpty));
        add(driveTo("into FLOWER", () -> flower()[1], true, AUTO_INTAKE_DRIVE_POWER).skipWhen(this::notEmpty));
        add(intakeDwell(AUTO_INTAKE_DWELL_S).skipWhen(this::notEmpty));
        add(driveTo("back out", () -> flower()[0], false, 1.0).skipWhen(this::isEmpty));

        add(driveToShootSpot().skipWhen(this::isEmpty));
        add(shoot(AUTO_SHOOT_STEP_TIMEOUT_S).skipWhen(this::isEmpty));
        add(park());
    }
}
