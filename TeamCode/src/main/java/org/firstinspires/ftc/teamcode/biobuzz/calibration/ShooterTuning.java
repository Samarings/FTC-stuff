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
import org.firstinspires.ftc.teamcode.biobuzz.Feeder;
import org.firstinspires.ftc.teamcode.biobuzz.FieldDraw;
import org.firstinspires.ftc.teamcode.biobuzz.PoseStorage;
import org.firstinspires.ftc.teamcode.biobuzz.Robot;
import org.firstinspires.ftc.teamcode.biobuzz.ShotSolver;
import org.firstinspires.ftc.teamcode.biobuzz.auto.AutoPoses;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Builds the distance -> RPM tables. Park anywhere, the solver shows the distance to the up
 * CELL; adjust RPM until shots go in, then log the point. Fit POLLEN_* / NECTAR_* tables in
 * RobotConstants from the logged points (smooth, increasing with distance).
 *
 * Start the robot at the ALLIANCE_WALL start spot (or run an AUTO first) so the pose is right.
 *
 * G1: drive (field-centric), A = auto-aim on/off, X/B in INIT = alliance
 * G2: B shooter on/off, dpad up/down RPM ±RPM_ADJUST_STEP (50), right stick button = copy table RPM,
 *     right trigger = FIRE (needs RPM ready only), Y = POLLEN/NECTAR,
 *     A = LOG point to /sdcard/FIRST/biobuzz_rpm_points.csv
 */
@TeleOp(name = "Shooter Tuning", group = "Calibration")
public class ShooterTuning extends OpMode {
    private static final String FILE = "/sdcard/FIRST/biobuzz_rpm_points.csv";

    private Robot robot;
    private Follower follower;
    private TelemetryManager telemetryM;
    private final Buttons buttons = new Buttons();
    private Alliance alliance = Alliance.RED;

    private boolean shooterOn = false, aimOn = false;
    private double rpm = 2500.0;
    private BallType ball = BallType.POLLEN;
    private final List<String> logged = new ArrayList<>();
    private double lastSec;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        FieldDraw.init();
        boolean fromAuto = PoseStorage.isFresh(1800.0);
        if (fromAuto) alliance = PoseStorage.alliance;
        robot = new Robot(hardwareMap, alliance, alliance.startUpCell(), false);
        follower = robot.localizer.follower();
        robot.localizer.setPose(fromAuto ? PoseStorage.pose : alliance.fromRed(AutoPoses.start(AutoPoses.Start.ALLIANCE_WALL)));
    }

    @Override
    public void init_loop() {
        boolean changed = false;
        if (buttons.pressed("x", gamepad1.x)) { alliance = Alliance.BLUE; changed = true; }
        if (buttons.pressed("b", gamepad1.b)) { alliance = Alliance.RED; changed = true; }
        if (changed) {
            robot.configure(alliance, alliance.startUpCell());
            robot.localizer.setPose(alliance.fromRed(AutoPoses.start(AutoPoses.Start.ALLIANCE_WALL)));
        }
        telemetry.addLine("Shooter Tuning. Alliance (X/B): " + alliance);
        telemetry.addLine("Robot must be at the ALLIANCE_WALL start spot (or run AUTO first).");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lastSec = getRuntime();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = Math.max(1e-3, Math.min(0.2, now - lastSec));
        lastSec = now;

        ShotSolver.Solution sol = robot.sense(now, ball, true, 0.0, true);

        if (buttons.pressed("g1a", gamepad1.a)) { aimOn = !aimOn; robot.aimer.reset(); }
        double sign = alliance == Alliance.RED ? 1 : -1;
        double turn = aimOn ? robot.aimer.update(sol, dt) : -gamepad1.right_stick_x * TELEOP_TURN_SCALE;
        follower.setTeleOpDrive(-gamepad1.left_stick_y * sign * 0.5, -gamepad1.left_stick_x * sign * 0.5, turn, false);

        if (buttons.pressed("g2b", gamepad2.b)) shooterOn = !shooterOn;
        if (buttons.pressed("g2up", gamepad2.dpad_up)) rpm += RPM_ADJUST_STEP;
        if (buttons.pressed("g2dn", gamepad2.dpad_down)) rpm -= RPM_ADJUST_STEP;
        if (buttons.pressed("g2rs", gamepad2.right_stick_button)) rpm = sol.rpm;
        if (buttons.pressed("g2y", gamepad2.y)) ball = ball == BallType.POLLEN ? BallType.NECTAR : BallType.POLLEN;
        rpm = Math.max(0, Math.min(rpmCap(), rpm));

        if (shooterOn) robot.shooter.setTargetRpm(rpm);
        else robot.shooter.off();

        boolean fire = gamepad2.right_trigger > 0.5 && robot.shooter.isReady();
        robot.feeder.setMode(fire ? Feeder.Mode.FEED : (gamepad2.left_trigger > 0.3 ? Feeder.Mode.COLLECT : Feeder.Mode.HOLD));
        robot.actuate(now, dt);

        if (buttons.pressed("g2a", gamepad2.a)) log(sol.distanceIn, rpm);

        telemetryM.addLine(String.format("%s  dist to %s CELL = %.1f in  (LL cluster %.1f in)", ball, sol.upCell,
                sol.distanceIn, robot.vision.ourCluster(sol.upCell).distanceIn));
        telemetryM.addLine(String.format("RPM set %.0f  meas %.0f  ready %b   table says %.0f",
                rpm, robot.shooter.getMeasuredRpm(), robot.shooter.isReady(), sol.rpm));
        if (robot.shooter.getFault() != null)
            telemetryM.addLine("⚠ SHOOTER FAULT: " + robot.shooter.getFault() + "  (B off clears it)");
        telemetryM.addLine(String.format("aim %s  err %.1f° (%s)", aimOn ? "ON" : "off", sol.aimErrDeg, sol.visionAim ? "LL" : "odom"));
        telemetryM.addLine("RPM cap " + rpmCap() + " (raise RPM_CAP, never above 6000, if far shots fall short)");
        telemetryM.addLine("Logged (dist_in, rpm, ball):");
        for (int i = Math.max(0, logged.size() - 8); i < logged.size(); i++) telemetryM.addLine("  " + logged.get(i));
        telemetryM.update(telemetry);
        Pose p = robot.localizer.getPose();
        FieldDraw.draw(p, robot.localizer.getLastAcceptedVisionPose(), sol.target);
    }

    private void log(double dist, double rpm) {
        String line = String.format(Locale.US, "%.1f,%.0f,%s", dist, rpm, ball);
        logged.add(line);
        try {
            File f = new File(FILE);
            boolean header = !f.exists();
            try (FileWriter w = new FileWriter(f, true)) {
                if (header) w.write("distance_in,rpm,ball\n");
                w.write(line + "\n");
            }
        } catch (Exception e) {
            logged.add("(file write failed: " + e.getMessage() + ")");
        }
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopMechanisms();
            robot.vision.stop();
        }
    }
}
