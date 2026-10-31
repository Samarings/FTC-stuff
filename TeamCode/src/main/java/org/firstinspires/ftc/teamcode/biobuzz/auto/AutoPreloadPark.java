package org.firstinspires.ftc.teamcode.biobuzz.auto;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.AUTO_SHOOT_STEP_TIMEOUT_S;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Baseline AUTO: leave the wall, shoot the 4 pre-loads at our up CELL, park in the LOADING
 * ZONE. LEAVE (3) + AUTO PARK (5), and with our partner's 4 pre-loads an AUTO TIP (20).
 */
@Autonomous(name = "BIOBUZZ Preload + Park", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class AutoPreloadPark extends BiobuzzAutoBase {

    @Override
    protected String routeName() { return "Preload + Park"; }

    @Override
    protected void buildRoute() {
        add(driveToShootSpot());
        add(shoot(AUTO_SHOOT_STEP_TIMEOUT_S));
        add(shootRetry());
        add(park());
    }
}
