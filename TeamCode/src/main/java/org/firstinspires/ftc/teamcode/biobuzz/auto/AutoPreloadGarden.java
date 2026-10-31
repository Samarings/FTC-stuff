package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Shoot the pre-loads, sweep our 4 GARDEN POLLEN along the audience wall (red: x 0-23 from
 * the A1 corner), shoot them, park. Each GARDEN ball left in place is worth 1 at match end;
 * a TIP is worth 20, so taking them is usually right. Best from the AUDIENCE start.
 *
 * Collection is skipped if the ball estimate says we still hold balls (G407, no sensor).
 */
@Autonomous(name = "BIOBUZZ Preload + Garden", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class AutoPreloadGarden extends BiobuzzAutoBase {

    @Override
    protected String routeName() { return "Preload + Garden"; }

    @Override
    protected List<Pose> routePosesRed() {
        return new ArrayList<>(Arrays.asList(AutoPoses.gardenSweep()));
    }

    @Override
    protected void buildRoute() {
        add(driveToShootSpot());
        add(shoot(AUTO_SHOOT_STEP_TIMEOUT_S));
        add(shootRetry());

        add(driveTo("to GARDEN", () -> AutoPoses.gardenSweep()[0], false, 1.0).skipWhen(this::notEmpty));
        add(driveTo("sweep GARDEN", () -> AutoPoses.gardenSweep()[1], true, AUTO_INTAKE_DRIVE_POWER).skipWhen(this::notEmpty));
        add(intakeDwell(0.5).skipWhen(this::notEmpty));

        add(driveToShootSpot().skipWhen(this::isEmpty));
        add(shoot(AUTO_SHOOT_STEP_TIMEOUT_S).skipWhen(this::isEmpty));
        add(park());
    }
}
