package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorMappingTest", group = "Debug")
public class MotorMappingTest extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // For this test, FORCE all directions to FORWARD
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("MotorMappingTest ready");
        telemetry.addLine("A = frontLeft,  B = frontRight");
        telemetry.addLine("X = backLeft,   Y = backRight");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.3;

            // Default: all off
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            if (gamepad1.a) {
                telemetry.addLine("frontLeft should spin FORWARD");
                frontLeft.setPower(power);
            } else if (gamepad1.b) {
                telemetry.addLine("frontRight should spin FORWARD");
                frontRight.setPower(power);
            } else if (gamepad1.x) {
                telemetry.addLine("backLeft should spin FORWARD");
                backLeft.setPower(power);
            } else if (gamepad1.y) {
                telemetry.addLine("backRight should spin FORWARD");
                backRight.setPower(power);
            }

            telemetry.update();
        }
    }
}
