package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.Alliance;
import org.firstinspires.ftc.teamcode.biobuzz.BallType;
import org.firstinspires.ftc.teamcode.biobuzz.Buttons;
import org.firstinspires.ftc.teamcode.biobuzz.CellSide;
import org.firstinspires.ftc.teamcode.biobuzz.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.Vision;
import org.firstinspires.ftc.teamcode.biobuzz.auto.AutoPoses;

/**
 * Stationary Limelight check (nothing moves). Shows which tag IDs are visible, each CELL
 * cluster's tx / ty / distance, the MegaTag2 pose, and why the localizer rejects frames.
 *
 * Use it to: confirm the pipeline sees all IDs 30-45; check the cluster distance against a
 * tape measure (3D vs ty fallback); check TIP detection by tipping the HIVE by hand on a
 * practice field (our cluster should switch CELLS and the TIP count go up).
 *
 * X / B = alliance (works during INIT and RUN).
 */
@TeleOp(name = "Vision Check", group = "Calibration")
public class VisionCheck extends OpMode {
    private Robot robot;
    private final Buttons buttons = new Buttons();
    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, alliance, alliance.startUpCell(), false);
        robot.localizer.setPose(alliance.fromRed(AutoPoses.start(AutoPoses.Start.ALLIANCE_WALL)));
    }

    @Override
    public void init_loop() { show(); }

    @Override
    public void loop() { show(); }

    private void show() {
        if (buttons.pressed("x", gamepad1.x)) { alliance = Alliance.BLUE; robot.configure(alliance, alliance.startUpCell()); }
        if (buttons.pressed("b", gamepad1.b)) { alliance = Alliance.RED; robot.configure(alliance, alliance.startUpCell()); }

        double now = getRuntime();
        robot.localizer.updatePoseOnly(now);
        robot.turret.read(now); // encoder only: the turret may be turned by hand
        robot.vision.update(robot.localizer.getHeadingDeg(), robot.turret.getAimAngleDeg(), now);
        robot.tips.update(robot.vision, now);
        robot.localizer.offerVision(robot.vision.getPoseFix(), robot.tips, false, now);

        telemetry.addLine("Vision Check (" + alliance + ")  Limelight " + (robot.vision.isConnected() ? "OK" : "NOT CONNECTED"));
        telemetry.addLine("Visible IDs: " + robot.vision.getVisibleIds());
        for (CellSide s : CellSide.values()) {
            Vision.ClusterView o = robot.vision.ourCluster(s), t = robot.vision.theirCluster(s);
            telemetry.addLine(String.format("OUR %s cluster: %d tags  center tx=%.1f° ty=%.1f° [%s]  dist=%.1f in (%s)",
                    s, o.tagCount, o.txDeg, o.tyDeg, o.centerMethod(), o.distanceIn, o.usedPose3d ? "3D" : "ty"));
            telemetry.addLine(String.format("    their %s cluster: %d tags", s, t.tagCount));
        }
        telemetry.addLine("Up CELL (tracker): " + robot.tips.getUpCell() + "  TIPS seen: " + robot.tips.getTipCount());
        Vision.PoseFix f = robot.vision.getPoseFix();
        telemetry.addLine(f == null ? "MegaTag2: none" : String.format(
                "MegaTag2 (Pedro): x=%.1f y=%.1f  tags=%d avgDist=%.2fm stale=%.0fms",
                f.pose.getX(), f.pose.getY(), f.tagCount, f.avgDistM, f.stalenessMs));
        telemetry.addLine("Last reject: " + robot.localizer.getLastReject());
        telemetry.addLine("Reject counts: " + robot.localizer.rejectSummary());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) robot.vision.stop();
    }
}
