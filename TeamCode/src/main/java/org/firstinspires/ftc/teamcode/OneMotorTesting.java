package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "OneMotorTesting", group = "Test")
public class OneMotorTesting extends OpMode {

    private DcMotorEx motor;

    // Change to your configuration name
    private static final String MOTOR_NAME = "testMotor";

    // Tuning knobs
    private static final boolean INVERT_STICK = true; // typical: pushing stick forward gives negative
    private static final double DEADZONE = 0.05;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // For speed telemetry (encoder)
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double stick = gamepad1.right_stick_y;
        if (INVERT_STICK) stick = -stick;

        // deadzone
        if (Math.abs(stick) < DEADZONE) stick = 0.0;

        // dynamic speed = stick position
        double power = Range.clip(stick, -1.0, 1.0);
        motor.setPower(power);

        // Speed telemetry (ticks/sec). If you want RPM, tell me your ticks/rev + gearing.
        double ticksPerSecond = motor.getVelocity();

        telemetry.addLine("=== One Motor Test (Right Stick Y) ===");
        telemetry.addData("Commanded Power", "%.2f", power);
        telemetry.addData("Velocity (ticks/sec)", "%.0f", ticksPerSecond);
        telemetry.addData("Encoder Position (ticks)", motor.getCurrentPosition());
        telemetry.update();
    }
}
