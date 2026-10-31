package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "IntakeAutoTest", group = "Auto")
public class IntakeAutoTest extends LinearOpMode {

    // ===== DRIVE MOTORS =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    // ===== MECHANISM MOTORS =====
    private DcMotor   intakeMotor;
    private DcMotor   feederMotor;
    private DcMotorEx shooterMotor;

    // ===== PINPOINT =====
    private GoBildaPinpointDriver pinpoint;

    // ===== LIMELIGHT 3A =====
    private Limelight3A limelight;
    private boolean limelightHasTarget      = false;
    private double  limelightTxDeg          = 0.0;
    private double  limelightTyDeg          = 0.0;
    private double  limelightDistanceMeters = 0.0;

    // Distance estimate constants – TUNE THESE FOR YOUR MOUNT
    private static final double LIMELIGHT_HEIGHT_M  = 0.50;  // camera center height
    private static final double TARGET_HEIGHT_M     = 0.90;  // target center height
    private static final double LIMELIGHT_PITCH_DEG = 20.0;  // up-tilt from horizontal

    // ===== MOVEMENT CONTROL CONSTANTS (from old RedClose) =====

    // "Good enough" tolerances
    private static final double POSITION_TOL_MM = 20.0;  // how close in mm
    private static final double HEADING_TOL_DEG = 3.0;   // how close in deg

    // Drive (forward/back) PID (error in mm -> power)
    private static final double DRIVE_KP = 0.004;
    private static final double DRIVE_KI = 0.0;
    private static final double DRIVE_KD = 0.0005;

    // Strafe (left/right) PID (error in mm -> power)
    private static final double STRAFE_KP = 0.004;
    private static final double STRAFE_KI = 0.0;
    private static final double STRAFE_KD = 0.0005;

    // Turn PID (error in deg -> rotate power)
    private static final double TURN_KP = 0.02;
    private static final double TURN_KI = 0.0;
    private static final double TURN_KD = 0.001;

    // Max turn power during corrections
    private static final double DRIVE_MAX_ROTATE = 0.5;

    // Stall detection: if commanded power is tiny for a bit, bail out
    private static final double MIN_MOVE_POWER = 0.05;
    private static final double MIN_TURN_POWER = 0.05;
    private static final double STALL_TIME_SEC = 0.25;

    private static final double PROGRESS_TIMEOUT_SEC = 0.55;  // currently unused

    // ===== DECELERATION NEAR TARGET ===== (currently unused)
    private static final double DECEL_DISTANCE_MM        = 290.0;
    private static final double MIN_DECEL_POWER_FRACTION = 0.33;

    // ===== LIMELIGHT TURN GAINS =====
    private static final double TAG_TURN_KP          = 0.02;
    private static final double TAG_MAX_TURN_POWER   = 0.6;
    private static final double TAG_AIM_DEADBAND_DEG = 1.0; // deg

    // ===== SHOOTER / INTAKE CONSTANTS =====
    // Shooter: goBILDA 5203 bare motor, 28 ticks/rev
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double AUTO_SHOOTER_RPM      = 2830; // tune

    private static final double INTAKE_POWER      = 1.0;
    private static final double INTAKE_LOW_POWER  = 0.75;  // low-speed intake
    private static final double FEEDER_POWER      = 0.8;
    // ===== FEEDER KICKBACK (pre-shot) =====
    private static final double FEEDER_KICKBACK_TIME_SEC     = 0.2; // seconds
    private static final double FEEDER_KICKBACK_POWER_MAG    = 1.0; // magnitude (0..1), applied opposite of feed direction
    private static final double FEEDER_KICKBACK_SETTLE_SEC   = 0.00; // optional tiny pause after kickback

    private static final double TOTAL_SHOOT_TIME_DEFAULT = 6.0;

    @Override
    public void runOpMode() {

        // ===== HARDWARE MAP =====
        frontLeft  = hardwareMap.get(DcMotor.class,  "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,  "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class,  "backLeft");
        backRight  = hardwareMap.get(DcMotor.class,  "backRight");

        intakeMotor  = hardwareMap.get(DcMotor.class,   "intake");
        feederMotor  = hardwareMap.get(DcMotor.class,   "feeder");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // === DRIVE DIRECTIONS – MATCH TELEOP ===
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Reverse intake same as TeleOp
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        feederMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== PINPOINT SETUP =====
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,   // X pod
                GoBildaPinpointDriver.EncoderDirection.REVERSED   // Y pod flipped
        );
        pinpoint.resetPosAndIMU();

        // ===== LIMELIGHT INIT =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // pipeline 0 in Limelight UI
        limelight.start();

        telemetry.addLine("BlueMulti READY (Pinpoint + Limelight + refactored shoot helpers)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        intakeOnForward();
        driveFeed(0.60, 850, 1, 0.3);
        driveBackwardWithLowIntake(470, 1, 4.0);


        // If you want pulsed intake shooting, call the separate function:
        // shootWithPulse(1.9, 3.6, 0.80, 1.00, 0.50); // pulse every 0.50s

        telemetry.addLine("AUTO DONE");
        telemetry.update();

        if (limelight != null) {
            limelight.stop();
        }
    }

    // ====== MOVEMENT HELPERS USING PINPOINT (AXIS-BY-AXIS) ======

    public void driveForward(double distanceMm, double maxPower, double timeoutSec) {
        driveAlongX(distanceMm, maxPower, timeoutSec);
    }

    public void driveBackward(double distanceMm, double maxPower, double timeoutSec) {
        driveAlongX(-distanceMm, maxPower, timeoutSec);
    }

    // drive backward while intake runs at low speed
    public void driveBackwardWithLowIntake(double distanceMm, double maxPower, double timeoutSec) {
        intakeOnLow();
        driveBackward(distanceMm, maxPower, timeoutSec);
        intakeOff();
    }

    /**
     * IMPORTANT: With this odometry config, positive Y from Pinpoint
     * corresponds to LEFT. (Callers handle sign.)
     */
    public void strafeRight(double distanceMm, double maxPower, double timeoutSec) {
        driveAlongY(distanceMm, maxPower, timeoutSec);
    }

    public void strafeLeft(double distanceMm, double maxPower, double timeoutSec) {
        driveAlongY(-distanceMm, maxPower, timeoutSec);
    }

    private void driveAlongX(double deltaXmm, double maxPower, double timeoutSec) {
        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();

        double startXmm        = startPose.getX(DistanceUnit.MM);
        double targetX         = startXmm + deltaXmm;
        double startHeadingDeg = startPose.getHeading(AngleUnit.DEGREES);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double lastTime      = timer.seconds();
        double posIntegral   = 0.0;
        double posLastError  = 0.0;
        double turnIntegral  = 0.0;
        double turnLastError = 0.0;

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double now = timer.seconds();
            double dt  = now - lastTime;
            if (dt <= 0) dt = 0.01;
            lastTime = now;

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double xMm     = pose.getX(DistanceUnit.MM);
            double yMm     = pose.getY(DistanceUnit.MM);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double errorX  = targetX - xMm;
            double absErrX = Math.abs(errorX);

            if (absErrX < POSITION_TOL_MM) break;

            // --- position PID (forward) ---
            posIntegral += errorX * dt;
            posIntegral = Range.clip(posIntegral, -500.0, 500.0);
            double posDerivative = (errorX - posLastError) / dt;
            posLastError = errorX;

            double cmdForward = DRIVE_KP * errorX
                    + DRIVE_KI * posIntegral
                    + DRIVE_KD * posDerivative;
            cmdForward = Range.clip(cmdForward, -maxPower, maxPower);

            // --- heading hold around start heading ---
            double hErrorDeg  = wrapAngleDeg(heading - startHeadingDeg);
            double absHErrDeg = Math.abs(hErrorDeg);

            double cmdRotate = 0.0;
            if (absHErrDeg > HEADING_TOL_DEG) {
                turnIntegral += hErrorDeg * dt;
                turnIntegral = Range.clip(turnIntegral, -200.0, 200.0);
                double turnDeriv = (hErrorDeg - turnLastError) / dt;
                turnLastError = hErrorDeg;

                cmdRotate = TURN_KP * hErrorDeg
                        + TURN_KI * turnIntegral
                        + TURN_KD * turnDeriv;
                cmdRotate = Range.clip(cmdRotate, -DRIVE_MAX_ROTATE, DRIVE_MAX_ROTATE);
            }

            if (Math.abs(cmdForward) < MIN_MOVE_POWER && Math.abs(cmdRotate) < MIN_TURN_POWER) {
                break;
            }

            mecanumDrive(cmdForward, 0.0, cmdRotate);

            telemetry.addLine("driveAlongX (PID)");
            telemetry.addData("X (cur/target)", "%.1f / %.1f", xMm, targetX);
            telemetry.addData("Y", "%.1f", yMm);
            telemetry.addData("Heading", "%.1f", heading);
            telemetry.addData("errorX", "%.1f", errorX);
            telemetry.addData("hErrorDeg", "%.1f", hErrorDeg);
            telemetry.addData("cmdForward", "%.3f", cmdForward);
            telemetry.addData("cmdRotate", "%.3f", cmdRotate);
            telemetry.update();
        }

        stopDrive();
    }

    private void driveAlongY(double deltaYmm, double maxPower, double timeoutSec) {
        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();

        double startXmm        = startPose.getX(DistanceUnit.MM);
        double startYmm        = startPose.getY(DistanceUnit.MM);
        double targetY         = startYmm + deltaYmm;
        double startHeadingDeg = startPose.getHeading(AngleUnit.DEGREES);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double lastTime      = timer.seconds();
        double posIntegral   = 0.0;
        double posLastError  = 0.0;
        double turnIntegral  = 0.0;
        double turnLastError = 0.0;

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            double now = timer.seconds();
            double dt  = now - lastTime;
            if (dt <= 0) dt = 0.01;
            lastTime = now;

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double xMm     = pose.getX(DistanceUnit.MM);
            double yMm     = pose.getY(DistanceUnit.MM);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double errorY  = targetY - yMm;
            double absErrY = Math.abs(errorY);

            double deltaX  = xMm - startXmm;  // fwd/back drift
            double deltaY  = yMm - startYmm;  // actual strafe

            if (absErrY < POSITION_TOL_MM) break;

            // --- position PID (strafe) ---
            posIntegral += errorY * dt;
            posIntegral = Range.clip(posIntegral, -500.0, 500.0);
            double posDerivative = (errorY - posLastError) / dt;
            posLastError = errorY;

            double cmdStrafe = STRAFE_KP * errorY
                    + STRAFE_KI * posIntegral
                    + STRAFE_KD * posDerivative;
            cmdStrafe = Range.clip(cmdStrafe, -maxPower, maxPower);

            // --- heading hold ---
            double hErrorDeg  = wrapAngleDeg(heading - startHeadingDeg);
            double absHErrDeg = Math.abs(hErrorDeg);

            double cmdRotate = 0.0;
            if (absHErrDeg > HEADING_TOL_DEG) {
                turnIntegral += hErrorDeg * dt;
                turnIntegral = Range.clip(turnIntegral, -200.0, 200.0);
                double turnDeriv = (hErrorDeg - turnLastError) / dt;
                turnLastError = hErrorDeg;

                cmdRotate = TURN_KP * hErrorDeg
                        + TURN_KI * turnIntegral
                        + TURN_KD * turnDeriv;
                cmdRotate = Range.clip(cmdRotate, -DRIVE_MAX_ROTATE, DRIVE_MAX_ROTATE);
            }

            if (Math.abs(cmdStrafe) < MIN_MOVE_POWER && Math.abs(cmdRotate) < MIN_TURN_POWER) {
                break;
            }

            mecanumDrive(0.0, cmdStrafe, cmdRotate);

            telemetry.addLine("driveAlongY (PID STRAFE)");
            telemetry.addData("Y (cur/target)", "%.1f / %.1f", yMm, targetY);
            telemetry.addData("errorY", "%.1f", errorY);
            telemetry.addData("deltaX drift", "%.1f", deltaX);
            telemetry.addData("deltaY moved", "%.1f", deltaY);
            telemetry.addData("Heading", "%.1f", heading);
            telemetry.addData("hErrorDeg", "%.1f", hErrorDeg);
            telemetry.addData("cmdStrafe", "%.3f", cmdStrafe);
            telemetry.addData("cmdRotate", "%.3f", cmdRotate);
            telemetry.update();
        }

        stopDrive();
    }

    /**
     * Turn to an absolute heading (degrees, Pinpoint frame) using simple P control.
     */
    public void turnToHeading(double targetHeadingDeg, double maxPower, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        int smallErrorCount = 0;

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double currentDeg = pose.getHeading(AngleUnit.DEGREES);
            double xMm        = pose.getX(DistanceUnit.MM);
            double yMm        = pose.getY(DistanceUnit.MM);

            double errorDeg  = wrapAngleDeg(currentDeg - targetHeadingDeg);
            double absErrDeg = Math.abs(errorDeg);

            if (absErrDeg < HEADING_TOL_DEG) {
                smallErrorCount++;
                if (smallErrorCount >= 2) break;
            } else {
                smallErrorCount = 0;
            }

            double cmdRotate = TURN_KP * errorDeg;
            cmdRotate = Range.clip(cmdRotate, -maxPower, maxPower);

            if (Math.abs(cmdRotate) < MIN_TURN_POWER) break;

            mecanumDrive(0.0, 0.0, cmdRotate);

            telemetry.addLine("turnToHeading (P-only)");
            telemetry.addData("Heading (cur/target)", "%.1f / %.1f", currentDeg, targetHeadingDeg);
            telemetry.addData("errorDeg", "%.1f", errorDeg);
            telemetry.addData("cmdRotate", "%.3f", cmdRotate);
            telemetry.addData("X", "%.1f", xMm);
            telemetry.addData("Y", "%.1f", yMm);
            telemetry.update();
        }

        stopDrive();
    }

    // =========================================================
    //                  LIMELIGHT LOCK + DISTANCE
    // =========================================================

    private double limelightLock(double maxTimeSec) {
        ElapsedTime timer = new ElapsedTime();
        double lastDistance = 0.0;

        updateLimelight();
        if (!limelightHasTarget || maxTimeSec <= 0) return 0.0;

        while (opModeIsActive() && timer.seconds() < maxTimeSec) {
            updateLimelight();
            if (!limelightHasTarget) break;

            double bearingDeg = limelightTxDeg; // +left / -right
            lastDistance = limelightDistanceMeters;

            if (Math.abs(bearingDeg) < TAG_AIM_DEADBAND_DEG) break;

            double turnCmd = Range.clip(TAG_TURN_KP * bearingDeg,
                    -TAG_MAX_TURN_POWER, TAG_MAX_TURN_POWER);

            mecanumDrive(0.0, 0.0, turnCmd);

            telemetry.addLine("Limelight aiming");
            telemetry.addData("tx", "%.1f", bearingDeg);
            telemetry.addData("turnCmd", "%.2f", turnCmd);
            telemetry.addData("distance (m)", "%.3f", lastDistance);
            telemetry.update();
        }

        stopDrive();
        return lastDistance;
    }

    private void updateLimelight() {
        limelightHasTarget      = false;
        limelightTxDeg          = 0.0;
        limelightTyDeg          = 0.0;
        limelightDistanceMeters = 0.0;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        limelightHasTarget = true;
        limelightTxDeg     = result.getTx();
        limelightTyDeg     = result.getTy();

        double totalAngleDeg = LIMELIGHT_PITCH_DEG + limelightTyDeg;
        double totalAngleRad = Math.toRadians(totalAngleDeg);
        double heightDiff    = TARGET_HEIGHT_M - LIMELIGHT_HEIGHT_M;

        if (Math.abs(Math.tan(totalAngleRad)) > 1e-3) {
            limelightDistanceMeters = heightDiff / Math.tan(totalAngleRad);
        } else {
            limelightDistanceMeters = 0.0;
        }
    }

    // =========================================================
    //         INTAKE / FEEDER / SHOOTING HELPERS
    // =========================================================

    public void intakeOnForward()  { intakeMotor.setPower(INTAKE_POWER); }
    public void intakeOnReverse()  { intakeMotor.setPower(-INTAKE_POWER); }
    public void intakeOnLow()      { intakeMotor.setPower(INTAKE_LOW_POWER); }
    public void intakeOff()        { intakeMotor.setPower(0.0); }

    public void feederOnForward()  { feederMotor.setPower(FEEDER_POWER); }
    public void feederOnReverse()  { feederMotor.setPower(-FEEDER_POWER); }
    public void feederOff()        { feederMotor.setPower(0.0); }

    public void shooterOn(double rpm) {
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksPerSec = rpm * SHOOTER_TICKS_PER_REV / 60.0;
        shooterMotor.setVelocity(ticksPerSec);
    }

    public void shooterOff() {
        shooterMotor.setVelocity(0);
        shooterMotor.setPower(0.0);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void feederKickbackBeforeShoot(double feederForwardPower) {
        // If feederForwardPower is 0, we still do a small reverse in "negative" direction by default.
        double sign = (Math.abs(feederForwardPower) > 1e-6) ? Math.signum(feederForwardPower) : 1.0;

        // Kick opposite direction of whatever you consider "forward feeding"
        double kickPower = -sign * FEEDER_KICKBACK_POWER_MAG;
        kickPower = Range.clip(kickPower, -1.0, 1.0);

        feederMotor.setPower(kickPower);
        sleep((long) (FEEDER_KICKBACK_TIME_SEC * 1000));

        feederMotor.setPower(0.0);
        if (FEEDER_KICKBACK_SETTLE_SEC > 0) {
            sleep((long) (FEEDER_KICKBACK_SETTLE_SEC * 1000));
        }
    }

    /**
     * NEW SIMPLE SHOOT:
     * Spins up shooter, then runs feeder+intake continuously at specified powers for maxShootTimeSec.
     */
    private void shoot(double spinupTimeSec,
                       double maxShootTimeSec,
                       double feederPower,
                       double intakePower) {

        feederPower = Range.clip(feederPower, -1.0, 1.0);
        intakePower = Range.clip(intakePower, -1.0, 1.0);

        shooterOn(AUTO_SHOOTER_RPM);
        sleep((long) (spinupTimeSec * 1000));

        // NEW: pre-shot kickback (intake OFF during kickback)
        intakeMotor.setPower(0.0);
        feederKickbackBeforeShoot(feederPower);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < maxShootTimeSec) {
            feederMotor.setPower(feederPower);
            intakeMotor.setPower(intakePower);

            telemetry.addLine("SHOOT (continuous + kickback)");
            telemetry.addData("t", "%.2f / %.2f", timer.seconds(), maxShootTimeSec);
            telemetry.addData("feederPower", "%.2f", feederPower);
            telemetry.addData("intakePower", "%.2f", intakePower);
            telemetry.addData("shooter vel (ticks/s)", "%.1f", shooterMotor.getVelocity());
            telemetry.update();
        }

        intakeMotor.setPower(0.0);
        feederMotor.setPower(0.0);
        shooterOff();
    }


    /**
     * NEW SEPARATE PULSE SHOOT:
     * Spins up shooter, runs feeder continuously, and pulses intake on/off at a given interval.
     *
     * intakePulseIntervalSec: seconds between toggles (e.g. 0.5 means toggle every 0.5s)
     * intakeOnPower: power when intake is ON (0..1). When OFF, intake power is 0.
     */
    private void shootWithPulse(double spinupTimeSec,
                                double maxShootTimeSec,
                                double feederPower,
                                double intakeOnPower,
                                double intakePulseIntervalSec) {

        feederPower   = Range.clip(feederPower, -1.0, 1.0);
        intakeOnPower = Range.clip(intakeOnPower, -1.0, 1.0);

        shooterOn(AUTO_SHOOTER_RPM);
        sleep((long) (spinupTimeSec * 1000));

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        boolean intakeOn = false;
        double lastToggle = 0.0;

        while (opModeIsActive() && timer.seconds() < maxShootTimeSec) {
            double t = timer.seconds();

            if (intakePulseIntervalSec > 0 && (t - lastToggle) >= intakePulseIntervalSec) {
                intakeOn = !intakeOn;
                lastToggle = t;
            }

            feederMotor.setPower(feederPower);
            intakeMotor.setPower(intakeOn ? intakeOnPower : 0.0);

            telemetry.addLine("SHOOT (pulse intake)");
            telemetry.addData("t", "%.2f / %.2f", t, maxShootTimeSec);
            telemetry.addData("feederPower", "%.2f", feederPower);
            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("intakeOnPower", "%.2f", intakeOnPower);
            telemetry.addData("pulseInterval", "%.2f", intakePulseIntervalSec);
            telemetry.addData("shooter vel (ticks/s)", "%.1f", shooterMotor.getVelocity());
            telemetry.update();
        }

        intakeMotor.setPower(0.0);
        feederMotor.setPower(0.0);
        shooterOff();
    }

    /**
     * Optional helper (kept): pulse intake only (no feeder/shooter changes).
     */
    public void pulseIntakeForSeconds(double totalTimeSec, double onTimeSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        boolean intakeOn = false;
        double lastToggle = 0.0;

        while (opModeIsActive() && timer.seconds() < totalTimeSec) {
            double t = timer.seconds();
            if (t - lastToggle >= onTimeSec) {
                intakeOn = !intakeOn;
                lastToggle = t;
            }

            intakeMotor.setPower(intakeOn ? INTAKE_POWER : 0.0);

            telemetry.addLine("pulseIntakeForSeconds");
            telemetry.addData("Time", "%.1f / %.1f", t, totalTimeSec);
            telemetry.addData("IntakeOn", intakeOn);
            telemetry.update();
        }

        intakeMotor.setPower(0.0);
    }

    // =========================================================
    //                 DRIVE + FEED (unchanged)
    // =========================================================

    private void driveFeed(double speed, double distanceMm,
                           double feederDelaySec, double feederRunSec) {

        double dir = Math.signum(speed);
        if (dir == 0) return;

        double maxPower = Math.abs(speed);
        double distance = Math.abs(distanceMm);

        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();
        double startXmm        = startPose.getX(DistanceUnit.MM);
        double startYmm        = startPose.getY(DistanceUnit.MM);
        double targetX         = startXmm + dir * distance;
        double startHeadingDeg = startPose.getHeading(AngleUnit.DEGREES);

        ElapsedTime moveTimer   = new ElapsedTime();
        ElapsedTime feederTimer = new ElapsedTime();
        double maxTimeSec       = 5.0 * (distance / 1000.0) + 2.0;

        boolean feederStarted = false;

        double lastTime      = moveTimer.seconds();
        double posIntegral   = 0.0;
        double posLastError  = 0.0;
        double turnIntegral  = 0.0;
        double turnLastError = 0.0;

        while (opModeIsActive() && moveTimer.seconds() < maxTimeSec) {
            double now = moveTimer.seconds();
            double dt  = now - lastTime;
            if (dt <= 0) dt = 0.01;
            lastTime = now;

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            double xMm     = pose.getX(DistanceUnit.MM);
            double yMm     = pose.getY(DistanceUnit.MM);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double errorX  = targetX - xMm;
            double absErrX = Math.abs(errorX);

            if (absErrX < POSITION_TOL_MM) break;

            posIntegral += errorX * dt;
            posIntegral = Range.clip(posIntegral, -500.0, 500.0);
            double posDerivative = (errorX - posLastError) / dt;
            posLastError = errorX;

            double cmdForward = DRIVE_KP * errorX
                    + DRIVE_KI * posIntegral
                    + DRIVE_KD * posDerivative;
            cmdForward = Range.clip(cmdForward, -maxPower, maxPower);

            double hErrorDeg  = wrapAngleDeg(heading - startHeadingDeg);
            double absHErrDeg = Math.abs(hErrorDeg);

            double cmdRotate = 0.0;
            if (absHErrDeg > HEADING_TOL_DEG) {
                turnIntegral += hErrorDeg * dt;
                turnIntegral = Range.clip(turnIntegral, -200.0, 200.0);
                double turnDeriv = (hErrorDeg - turnLastError) / dt;
                turnLastError = hErrorDeg;

                cmdRotate = TURN_KP * hErrorDeg
                        + TURN_KI * turnIntegral
                        + TURN_KD * turnDeriv;
                cmdRotate = Range.clip(cmdRotate, -DRIVE_MAX_ROTATE, DRIVE_MAX_ROTATE);
            }

            if (Math.abs(cmdForward) < MIN_MOVE_POWER && Math.abs(cmdRotate) < MIN_TURN_POWER) {
                break;
            }

            double t = feederTimer.seconds();
            if (!feederStarted && t >= feederDelaySec) feederStarted = true;

            if (feederStarted && t <= feederDelaySec + feederRunSec) {
                feederMotor.setPower(FEEDER_POWER);
            } else {
                feederMotor.setPower(0.0);
            }

            mecanumDrive(cmdForward, 0.0, cmdRotate);

            telemetry.addLine("DRIVE + FEED (PID)");
            telemetry.addData("Distance error X (mm)", "%.1f", errorX);
            telemetry.addData("headingError", "%.1f", hErrorDeg);
            telemetry.addData("cmdForward", "%.3f", cmdForward);
            telemetry.addData("cmdRotate", "%.3f", cmdRotate);
            telemetry.addData("feederOn", feederStarted && t <= feederDelaySec + feederRunSec);
            telemetry.addData("X", "%.1f", xMm);
            telemetry.addData("Y", "%.1f", yMm);
            telemetry.update();
        }

        stopDrive();
        feederMotor.setPower(0.0);
    }

    // ====== LOW-LEVEL DRIVE HELPERS ======

    private void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);

        leftFrontPower  = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower   = (forward - strafe + rotate) / denominator;
        rightBackPower  = (forward + strafe - rotate) / denominator;

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }

    private void stopDrive() {
        mecanumDrive(0.0, 0.0, 0.0);
    }

    // ====== ANGLE UTIL ======

    private static double wrapAngleDeg(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle <  -180.0) angle += 360.0;
        return angle;
    }
}
