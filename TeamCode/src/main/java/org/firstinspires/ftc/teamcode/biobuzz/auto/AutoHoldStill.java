package org.firstinspires.ftc.teamcode.biobuzz.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.biobuzz.Buttons;
import org.firstinspires.ftc.teamcode.biobuzz.CellSide;
import org.firstinspires.ftc.teamcode.biobuzz.PoseStorage;
import org.firstinspires.ftc.teamcode.biobuzz.Robot;

/**
 * Use this in matches until the real AUTO routes are tested.
 *
 * G305: every team must INIT an AUTO OpMode; teams without an AUTO should run a basic one
 * that queues their TeleOp. This one NEVER moves anything (no drive, no flywheel, no intake).
 * It only seeds the field pose from the chosen start spot and hands it to TeleOp through
 * PoseStorage, so "shoot from anywhere" starts with the right position.
 *
 * INIT (gamepad 1): X / B alliance, dpad up/down start spot, Y flips the starting up CELL.
 */
@Autonomous(name = "BIOBUZZ Hold Still (no AUTO)", group = "BIOBUZZ", preselectTeleOp = "BIOBUZZ TeleOp")
public class AutoHoldStill extends OpMode {
    private Robot robot;
    private final Buttons buttons = new Buttons();
    private Alliance alliance = Alliance.RED;
    private AutoPoses.Start startPos = AutoPoses.Start.ALLIANCE_WALL;
    private boolean flipUpCell = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, alliance, upCell(), true);
        robot.shooter.off();
        robot.localizer.resetImuAndSetPose(startPose());
    }

    @Override
    public void init_loop() {
        boolean changed = false;
        if (buttons.pressed("x", gamepad1.x)) { alliance = Alliance.BLUE; changed = true; }
        if (buttons.pressed("b", gamepad1.b)) { alliance = Alliance.RED; changed = true; }
        if (buttons.pressed("up", gamepad1.dpad_up)) { startPos = cycle(startPos, 1); changed = true; }
        if (buttons.pressed("dn", gamepad1.dpad_down)) { startPos = cycle(startPos, -1); changed = true; }
        if (buttons.pressed("y", gamepad1.y)) { flipUpCell = !flipUpCell; changed = true; }
        if (changed) {
            robot.configure(alliance, upCell());
            robot.localizer.setPose(startPose());
        }
        robot.localizer.updatePoseOnly(getRuntime());
        Pose p = robot.localizer.getPose();
        telemetry.addLine("BIOBUZZ Hold Still: the robot will NOT move in AUTO.");
        telemetry.addLine("Alliance (X/B): " + alliance + "   Start spot (dpad): " + startPos);
        telemetry.addLine("Up CELL at start (Y flips): " + upCell());
        telemetry.addLine(String.format("Seeded pose: (%.1f, %.1f, %.0f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read odometry only; never command the drive or mechanisms.
        robot.localizer.updatePoseOnly(getRuntime());
        robot.savePose();
        telemetry.addLine("Holding still. TeleOp is queued.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.savePose();
            robot.vision.stop();
        }
    }

    private CellSide upCell() {
        CellSide c = alliance.startUpCell();
        return flipUpCell ? c.other() : c;
    }

    private Pose startPose() {
        return alliance.fromRed(AutoPoses.start(startPos));
    }

    private static AutoPoses.Start cycle(AutoPoses.Start s, int dir) {
        AutoPoses.Start[] v = AutoPoses.Start.values();
        return v[((s.ordinal() + dir) % v.length + v.length) % v.length];
    }
}
