package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.biobuzz.BallType;
import org.firstinspires.ftc.teamcode.biobuzz.Buttons;
import org.firstinspires.ftc.teamcode.biobuzz.FieldDraw;
import org.firstinspires.ftc.teamcode.biobuzz.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.RobotConstants;
import org.firstinspires.ftc.teamcode.biobuzz.Vision;
import org.firstinspires.ftc.teamcode.biobuzz.auto.AutoPoses;

/**
 * Odometry vs Limelight. Drive around and watch both poses on the Panels field view.
 *
 * Checks to run (brief, "Localization -> Testing"):
 *  1. Frame check: park at a known spot facing +x (toward the blue wall). The raw MegaTag2
 *     pose and heading below must match the Pedro pose. If they're rotated, fix
 *     LL_TO_PEDRO_ROTATION_DEG (try 90 / -90 / 180).
 *  2. Spin test: spin in place 10 times (G1 right stick). Position should barely move;
 *     if it circles, the Pinpoint pod offsets in pedroPathing/Constants.java are wrong.
 *  3. Drift: drive a loop and return to the start mark; read the drift.
 *  4. Corrections: start ~6 in off from the seeded pose; corrections should pull it back
 *     within a few seconds without jerking. G1 B toggles corrections on/off.
 *
 * Start at the ALLIANCE_WALL start spot. INIT: X/B alliance. G1 BACK+START (1 s) re-seeds.
 */
@TeleOp(name = "Localization Test", group = "Calibration")
public class LocalizationTest extends OpMode {
    private Robot robot;
    private Follower follower;
    private TelemetryManager telemetryM;
    private final Buttons buttons = new Buttons();
    private Alliance alliance = Alliance.RED;
    private Pose seeded;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        FieldDraw.init();
        robot = new Robot(hardwareMap, alliance, alliance.startUpCell(), false);
        follower = robot.localizer.follower();
        seeded = alliance.fromRed(AutoPoses.start(AutoPoses.Start.ALLIANCE_WALL));
        robot.localizer.resetImuAndSetPose(seeded);
    }

    @Override
    public void init_loop() {
        boolean changed = false;
        if (buttons.pressed("x", gamepad1.x)) { alliance = Alliance.BLUE; changed = true; }
        if (buttons.pressed("b", gamepad1.b)) { alliance = Alliance.RED; changed = true; }
        if (changed) {
            robot.configure(alliance, alliance.startUpCell());
            seeded = alliance.fromRed(AutoPoses.start(AutoPoses.Start.ALLIANCE_WALL));
            robot.localizer.setPose(seeded);
        }
        telemetry.addLine("Localization Test. Alliance (X/B): " + alliance);
        telemetry.addLine(String.format("Seeded at ALLIANCE_WALL start: (%.1f, %.1f, %.0f°)",
                seeded.getX(), seeded.getY(), Math.toDegrees(seeded.getHeading())));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        robot.sense(now, BallType.POLLEN, false, 0.0, true);

        if (buttons.pressed("g1b", gamepad1.b)) VISION_CORRECTIONS_ENABLED = !VISION_CORRECTIONS_ENABLED;
        if (buttons.heldFor("reseed", gamepad1.back && gamepad1.start, 1.0, now)) robot.localizer.setPose(seeded);

        double sign = alliance == Alliance.RED ? 1 : -1;
        follower.setTeleOpDrive(-gamepad1.left_stick_y * sign * 0.6, -gamepad1.left_stick_x * sign * 0.6,
                -gamepad1.right_stick_x * 0.6, false);

        Pose p = robot.localizer.getPose();
        telemetryM.addLine("Corrections (G1 B): " + (VISION_CORRECTIONS_ENABLED ? "ON" : "OFF"));
        for (String l : robot.localizer.describe(now)) telemetryM.addLine(l);
        telemetryM.addLine(String.format("Drift from seed: dx=%.1f dy=%.1f dh=%.1f°",
                p.getX() - seeded.getX(), p.getY() - seeded.getY(),
                Math.toDegrees(p.getHeading() - seeded.getHeading())));
        Vision.PoseFix fix = robot.vision.getPoseFix();
        if (fix != null) {
            telemetryM.addLine(String.format("RAW MT2 (Pedro): x=%.1f y=%.1f yaw=%.1f°  tags=%d avgDist=%.2fm lat=%.0fms",
                    fix.pose.getX(), fix.pose.getY(), Math.toDegrees(fix.pose.getHeading()),
                    fix.tagCount, fix.avgDistM, fix.totalLatencyMs));
        } else {
            telemetryM.addLine("RAW MT2: none");
        }
        telemetryM.addLine("Tags: " + robot.vision.getVisibleIds());
        telemetryM.addLine("Pose source owner: Pedro follower + Pinpoint (Constants.java pod offsets)");
        for (String w : RobotConstants.placeholderWarnings()) telemetryM.addLine("⚠ " + w);
        telemetryM.update(telemetry);
        FieldDraw.draw(p, fix != null ? fix.pose : null, null);
    }

    @Override
    public void stop() {
        VISION_CORRECTIONS_ENABLED = true; // don't leave corrections off for the next OpMode
        if (robot != null) robot.vision.stop();
    }
}
