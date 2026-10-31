package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "PoseTest ", group = "Test")
public class PoseTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private Pose2D pose;

    private boolean aPrev = false;

    @Override
    public void runOpMode() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Same baseline setup as your other opmodes
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        telemetry.setMsTransmissionInterval(25);
        telemetry.addLine("PinpointPoseOnly (INCHES): shows pose. G1.A = reset pose.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            pinpoint.update();
            pose = pinpoint.getPosition();

            double xIn = pose.getX(DistanceUnit.INCH);
            double yIn = pose.getY(DistanceUnit.INCH);
            double headingDeg = pose.getHeading(AngleUnit.DEGREES);

            // G1.A rising edge -> reset (safe across SDK variants)
            if (gamepad1.a && !aPrev) {
                pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
                try { pinpoint.resetPosAndIMU(); } catch (Exception ignored) {}
            }
            aPrev = gamepad1.a;

            telemetry.addData("X (in)", "%.3f", xIn);
            telemetry.addData("Y (in)", "%.3f", yIn);
            telemetry.addData("Heading (deg)", "%.2f", headingDeg);
            telemetry.update();
        }
    }
}
