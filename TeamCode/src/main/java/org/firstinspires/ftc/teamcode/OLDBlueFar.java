package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

import java.util.List;

@Autonomous(name = "OLDbluefar", group = "Auto")
public class OLDBlueFar extends LinearOpMode {

    // ===== DRIVE MOTORS =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    // ===== SHOOTER KICK-BRAKE (reverse pulse at end of shooting) =====
    private static final double SHOOTER_BRAKE_POWER    = 1.0;
    private static final double SHOOTER_BRAKE_TIME_SEC = 0.4;
    private static final double SHOOTER_BRAKE_MIN_RPM  = 100.0;

    // ===== MECHANISM MOTORS =====
    private DcMotor   intakeMotor;
    private DcMotor   feederMotor;
    private DcMotorEx shooterMotor;

    // driveFeed decel defaults
    private static final double DECEL_DISTANCE_MM        = 290.0;
    private static final double MIN_DECEL_POWER_FRACTION = 0.33;

    // ===== PINPOINT =====
    private GoBildaPinpointDriver pinpoint;

    // ===== LIMELIGHT 3A =====
    private Limelight3A limelight;
    private boolean limelightHasTarget      = false;
    private double  limelightTxDeg          = 0.0;
    private double  limelightTyDeg          = 0.0;
    private double  limelightDistanceMeters = 0.0;
    private int     limelightTagId          = -1; // NEW

    // Save last lock (so shoot() can choose near vs far)
    private double lastLockedDistanceM = 0.0;
    private int    lastLockedTagId     = -1;

    // Distance estimate constants – TUNE FOR YOUR MOUNT
    private static final double LIMELIGHT_HEIGHT_M  = 0.50;
    private static final double TARGET_HEIGHT_M     = 0.90;
    private static final double LIMELIGHT_PITCH_DEG = 20.0;

    // ===== MOVEMENT CONTROL CONSTANTS =====
    private static final double POSITION_TOL_MM = 20.0;
    private static final double HEADING_TOL_DEG = 3.0;

    private static final double DRIVE_KP = 0.004;
    private static final double DRIVE_KI = 0.0;
    private static final double DRIVE_KD = 0.0005;

    private static final double STRAFE_KP = 0.004;
    private static final double STRAFE_KI = 0.0;
    private static final double STRAFE_KD = 0.0005;

    private static final double TURN_KP = 0.02;
    private static final double TURN_KI = 0.0;
    private static final double TURN_KD = 0.001;

    private static final double DRIVE_MAX_ROTATE = 0.5;

    private static final double MIN_MOVE_POWER = 0.05;
    private static final double MIN_TURN_POWER = 0.05;
    private static final double STALL_TIME_SEC = 0.25;

    // ===== LIMELIGHT TURN GAINS =====
    private static final double TAG_TURN_KP          = 0.02;
    private static final double TAG_MAX_TURN_POWER   = 0.6;
    private static final double TAG_AIM_DEADBAND_DEG = 1.0;

    // ===== FAR SHOT PACKAGE (from TeleOp) =====
    private static final double FAR_DISTANCE_THRESHOLD_M = 1.75;
    private static final double FAR_AUTO_RPM             = 4000.0;
    private static final double TAG_AIM_BIAS_DEG         = 2.0;  // Tag 20 = +2, Tag 24 = -2 (only when far)

    // ===== SHOOTER / INTAKE CONSTANTS =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // goBILDA 5203 series

    // This is your current "close" auto rpm target
    private static final double AUTO_SHOOTER_RPM      = 3200.0;

    private static final double INTAKE_POWER      = 1.0;
    private static final double INTAKE_LOW_POWER  = 0.75;
    private static final double FEEDER_POWER      = 0.6;

    // ===== FEEDER KICKBACK (pre-shot) =====
    private static final double FEEDER_KICKBACK_TIME_SEC     = 0.3;
    private static final double FEEDER_KICKBACK_POWER_MAG    = 1.0;
    private static final double FEEDER_KICKBACK_SETTLE_SEC   = 0.00;

    // ===== SHOOTER PIDF (TeleOp Near/Far) =====
    private static final double SHOOTER_P_NEAR = 9.3;
    private static final double SHOOTER_I_NEAR = 0.0;
    private static final double SHOOTER_D_NEAR = 0.4;
    private static final double SHOOTER_F_NEAR = 13.65;

    private static final double SHOOTER_P_FAR  = 8.3;
    private static final double SHOOTER_I_FAR  = 0.0;
    private static final double SHOOTER_D_FAR  = 0.3;
    private static final double SHOOTER_F_FAR  = 13.65;

    private boolean shooterPidIsFar = false;

    // ===== SHOOTER RAMP (RPM setpoint ramp) =====
    private double shooterRpmCmd = 0.0;
    private static final double SHOOTER_RAMP_UP_RPM_PER_SEC_NEAR   = 2700.0;
    private static final double SHOOTER_RAMP_UP_RPM_PER_SEC_FAR    = 3500.0;
    private static final double SHOOTER_RAMP_DOWN_RPM_PER_SEC_NEAR = 9900.0;
    private static final double SHOOTER_RAMP_DOWN_RPM_PER_SEC_FAR  = 9900.0;

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

        // Reverse intake/feeder same as TeleOp
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        feederMotor.setDirection(DcMotor.Direction.REVERSE);

        // Brakes
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter setup
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start NEAR PIDF
        applyShooterPIDF(false);
        shooterPidIsFar = false;

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
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("BlueMulti READY (Pinpoint + Limelight + NEAR/FAR PIDF + FAR bias + FAR 4000rpm)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ===== AUTO SEQUENCE (unchanged) =====
        intakeOnLow();
        strafeRight(320, 0.8, 3.0);
        intakeOff();
        turnToHeading(-67.0, 0.35, 2.0);

        double distanceM = limelightLock(0.8);
        telemetry.addLine("After Limelight lock");
        telemetry.addData("Estimated distance (m)", "%.3f", distanceM);
        telemetry.addData("TagId", "%d", lastLockedTagId);
        telemetry.addData("FAR?", "%s", (distanceM > FAR_DISTANCE_THRESHOLD_M));
        telemetry.addData("Shooter RPM target", "%.1f", selectShooterTargetRpm(distanceM));
        telemetry.update();

        shoot(2.0, 2.2, 0.80, 0.80);

        turnToHeading(0, 0.3, 1.2);
        strafeRight(340, 1.0, 3);

        if (limelight != null) limelight.stop();
    }

    // =========================================================
    //                  FAR PACKAGE HELPERS
    // =========================================================
    private double selectShooterTargetRpm(double distanceM) {
        return (distanceM > FAR_DISTANCE_THRESHOLD_M) ? FAR_AUTO_RPM : AUTO_SHOOTER_RPM;
    }

    private double getAimBiasDegForTag(int tagId) {
        if (tagId == 20) return  TAG_AIM_BIAS_DEG;
        if (tagId == 24) return -TAG_AIM_BIAS_DEG;
        return 0.0;
    }

    // =========================================================
    //                  SHOOTER PIDF + RAMP HELPERS
    // =========================================================
    private void shooterKickBrakePulse() {
        double velTicksPerSec = shooterMotor.getVelocity();
        double rpmMeas = Math.abs(velTicksPerSec) * 60.0 / SHOOTER_TICKS_PER_REV;

        if (rpmMeas < SHOOTER_BRAKE_MIN_RPM) return;

        double sign = (Math.abs(velTicksPerSec) > 1e-6) ? Math.signum(velTicksPerSec) : 1.0;

        double reversePower = -sign * SHOOTER_BRAKE_POWER;
        reversePower = Range.clip(reversePower, -1.0, 1.0);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(reversePower);
        sleep((long) (SHOOTER_BRAKE_TIME_SEC * 1000));

        shooterMotor.setPower(0.0);
    }

    private void applyShooterPIDF(boolean far) {
        try {
            if (far) {
                shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P_FAR, SHOOTER_I_FAR, SHOOTER_D_FAR, SHOOTER_F_FAR);
            } else {
                shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P_NEAR, SHOOTER_I_NEAR, SHOOTER_D_NEAR, SHOOTER_F_NEAR);
            }
        } catch (Exception ignored) {}
    }

    private void setShooterVelocityRpm(double rpm) {
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksPerSec = rpm * SHOOTER_TICKS_PER_REV / 60.0;
        shooterMotor.setVelocity(ticksPerSec);
    }

    private void rampShooterToward(double targetRpm, double dt, boolean farMode) {
        double rampUp   = farMode ? SHOOTER_RAMP_UP_RPM_PER_SEC_FAR   : SHOOTER_RAMP_UP_RPM_PER_SEC_NEAR;
        double rampDown = farMode ? SHOOTER_RAMP_DOWN_RPM_PER_SEC_FAR : SHOOTER_RAMP_DOWN_RPM_PER_SEC_NEAR;

        double maxDelta = ((targetRpm > shooterRpmCmd) ? rampUp : rampDown) * dt;
        shooterRpmCmd = stepToward(shooterRpmCmd, targetRpm, maxDelta);

        if (shooterRpmCmd < 1.0) {
            shooterMotor.setPower(0.0);
            return;
        }
        setShooterVelocityRpm(shooterRpmCmd);
    }

    // =========================================================
    //                  LIMELIGHT LOCK + DISTANCE + TAGID + FAR BIAS
    // =========================================================
    private double limelightLock(double maxTimeSec) {
        ElapsedTime timer = new ElapsedTime();
        double lastDistance = 0.0;
        int lastTag = -1;

        updateLimelight();
        if (!limelightHasTarget || maxTimeSec <= 0) {
            lastLockedDistanceM = 0.0;
            lastLockedTagId = -1;
            return 0.0;
        }

        while (opModeIsActive() && timer.seconds() < maxTimeSec) {
            updateLimelight();
            if (!limelightHasTarget) break;

            lastDistance = limelightDistanceMeters;
            lastTag = limelightTagId;

            boolean farMode = (lastDistance > FAR_DISTANCE_THRESHOLD_M);

            double aimOffsetDeg = 0.0;
            if (farMode && lastTag != -1) {
                aimOffsetDeg = getAimBiasDegForTag(lastTag);
            }

            double bearingDeg = limelightTxDeg - aimOffsetDeg;

            if (Math.abs(bearingDeg) < TAG_AIM_DEADBAND_DEG) break;

            double turnCmd = Range.clip(TAG_TURN_KP * bearingDeg,
                    -TAG_MAX_TURN_POWER, TAG_MAX_TURN_POWER);

            mecanumDrive(0.0, 0.0, turnCmd);

            telemetry.addLine("Limelight aiming");
            telemetry.addData("tagId", "%d", lastTag);
            telemetry.addData("farMode", farMode);
            telemetry.addData("aimOffset", "%.1f", aimOffsetDeg);
            telemetry.addData("txRaw/txBiased", "%.2f / %.2f", limelightTxDeg, bearingDeg);
            telemetry.addData("turnCmd", "%.2f", turnCmd);
            telemetry.addData("distance (m)", "%.3f", lastDistance);
            telemetry.update();
        }

        stopDrive();

        lastLockedDistanceM = lastDistance;
        lastLockedTagId     = lastTag;

        return lastDistance;
    }

    private void updateLimelight() {
        limelightHasTarget      = false;
        limelightTxDeg          = 0.0;
        limelightTyDeg          = 0.0;
        limelightDistanceMeters = 0.0;
        limelightTagId          = -1;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        limelightHasTarget = true;
        limelightTxDeg     = result.getTx();
        limelightTyDeg     = result.getTy();

        // Tag ID
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            limelightTagId = fiducials.get(0).getFiducialId();
        }

        // Distance from pitch+ty
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

    public void shooterOff() {
        shooterRpmCmd = 0.0;
        try { shooterMotor.setVelocity(0); } catch (Exception ignored) {}
        shooterMotor.setPower(0.0);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void feederKickbackBeforeShoot(double feederForwardPower) {
        double sign = (Math.abs(feederForwardPower) > 1e-6) ? Math.signum(feederForwardPower) : 1.0;

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
     * SHOOT:
     * Kickback -> ramp shooter to NEAR or FAR rpm (auto chosen from last lock) -> feed -> brake pulse -> stop.
     */
    private void shoot(double spinupTimeSec,
                       double maxShootTimeSec,
                       double feederPower,
                       double intakePower) {

        feederPower = Range.clip(feederPower, -1.0, 1.0);
        intakePower = Range.clip(intakePower, -1.0, 1.0);

        // Decide mode based on last Limelight lock result
        boolean farMode = (lastLockedDistanceM > FAR_DISTANCE_THRESHOLD_M);
        double targetRpm = selectShooterTargetRpm(lastLockedDistanceM);

        // Ensure PIDF matches mode
        if (farMode != shooterPidIsFar) {
            applyShooterPIDF(farMode);
            shooterPidIsFar = farMode;
        }

        // 1) intake OFF for kickback
        intakeMotor.setPower(0.0);

        // 2) Kickback BEFORE shooter acceleration
        feederKickbackBeforeShoot(feederPower);

        // 3) Spin up shooter using RAMP + PIDF
        ElapsedTime spinTimer = new ElapsedTime();
        double lastT = 0.0;

        while (opModeIsActive() && spinTimer.seconds() < spinupTimeSec) {
            double t = spinTimer.seconds();
            double dt = t - lastT;
            if (dt <= 0) dt = 0.02;
            if (dt > 0.2) dt = 0.02;
            lastT = t;

            rampShooterToward(targetRpm, dt, farMode);

            double measTps = shooterMotor.getVelocity();
            double measRpm = measTps * 60.0 / SHOOTER_TICKS_PER_REV;

            telemetry.addLine("SHOOT SPINUP (PIDF + ramp)");
            telemetry.addData("mode", farMode ? "FAR" : "NEAR");
            telemetry.addData("lockedDist/tag", "%.2f / %d", lastLockedDistanceM, lastLockedTagId);
            telemetry.addData("target/cmd/meas rpm", "%.0f / %.0f / %.0f", targetRpm, shooterRpmCmd, measRpm);
            telemetry.update();

            idle();
        }

        // 4) Feed while continuing to hold RPM
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double lastLoop = 0.0;

        while (opModeIsActive() && timer.seconds() < maxShootTimeSec) {
            double t = timer.seconds();
            double dt = t - lastLoop;
            if (dt <= 0) dt = 0.02;
            if (dt > 0.2) dt = 0.02;
            lastLoop = t;

            rampShooterToward(targetRpm, dt, farMode);

            feederMotor.setPower(feederPower);
            intakeMotor.setPower(intakePower);

            telemetry.addLine("SHOOTING (PIDF + ramp hold)");
            telemetry.addData("mode", farMode ? "FAR" : "NEAR");
            telemetry.addData("t", "%.2f / %.2f", t, maxShootTimeSec);
            telemetry.update();

            idle();
        }

        intakeMotor.setPower(0.0);
        feederMotor.setPower(0.0);

        // optional tiny pause so no ring is half-engaged when braking
        sleep(30);

        // kick-brake the flywheel, then fully stop
        shooterKickBrakePulse();
        shooterOff();
    }

    // =========================================================
    //                 DRIVE + FEED (unchanged)
    // =========================================================
    public void driveForward(double distanceMm, double maxPower, double timeoutSec) {
        driveAlongX(distanceMm, maxPower, timeoutSec);
    }

    public void driveBackward(double distanceMm, double maxPower, double timeoutSec) {
        driveAlongX(-distanceMm, maxPower, timeoutSec);
    }

    public void driveBackwardWithLowIntake(double distanceMm, double maxPower, double timeoutSec) {
        intakeOnLow();
        driveBackward(distanceMm, maxPower, timeoutSec);
        intakeOff();
    }

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

            if (Math.abs(cmdForward) < MIN_MOVE_POWER && Math.abs(cmdRotate) < MIN_TURN_POWER) break;

            mecanumDrive(cmdForward, 0.0, cmdRotate);
            telemetry.update();
        }

        stopDrive();
    }

    private void driveAlongY(double deltaYmm, double maxPower, double timeoutSec) {
        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();

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

            double yMm     = pose.getY(DistanceUnit.MM);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double errorY  = targetY - yMm;
            double absErrY = Math.abs(errorY);

            if (absErrY < POSITION_TOL_MM) break;

            posIntegral += errorY * dt;
            posIntegral = Range.clip(posIntegral, -500.0, 500.0);
            double posDerivative = (errorY - posLastError) / dt;
            posLastError = errorY;

            double cmdStrafe = STRAFE_KP * errorY
                    + STRAFE_KI * posIntegral
                    + STRAFE_KD * posDerivative;
            cmdStrafe = Range.clip(cmdStrafe, -maxPower, maxPower);

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

            if (Math.abs(cmdStrafe) < MIN_MOVE_POWER && Math.abs(cmdRotate) < MIN_TURN_POWER) break;

            mecanumDrive(0.0, cmdStrafe, cmdRotate);
            telemetry.update();
        }

        stopDrive();
    }

    public void turnToHeading(double targetHeadingDeg, double maxPower, double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        int smallErrorCount = 0;

        while (opModeIsActive() && timer.seconds() < timeoutSec) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double currentDeg = pose.getHeading(AngleUnit.DEGREES);

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
            telemetry.update();
        }

        stopDrive();
    }

    // ===== driveFeed with defaults =====
    private void driveFeed(double speed, double distanceMm,
                           double feederDelaySec, double feederRunSec) {
        driveFeed(speed, distanceMm, feederDelaySec, feederRunSec,
                DECEL_DISTANCE_MM, MIN_DECEL_POWER_FRACTION);
    }

    private void driveFeed(double speed, double distanceMm,
                           double feederDelaySec, double feederRunSec,
                           double decelDistanceMm, double minPowerFraction) {

        double dir = Math.signum(speed);
        if (dir == 0) return;

        double maxPower = Math.abs(speed);
        double distance = Math.abs(distanceMm);

        decelDistanceMm  = Math.max(0.0, decelDistanceMm);
        minPowerFraction = Range.clip(minPowerFraction, 0.0, 1.0);

        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();
        double startXmm        = startPose.getX(DistanceUnit.MM);
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

            // STEP DECEL: immediate clamp once inside last decelDistanceMm
            double effMaxPower = maxPower;
            if (decelDistanceMm > 1e-6 && absErrX < decelDistanceMm) {
                effMaxPower = maxPower * minPowerFraction;
            }
            cmdForward = Range.clip(cmdForward, -effMaxPower, effMaxPower);

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

            if (Math.abs(cmdForward) < MIN_MOVE_POWER && Math.abs(cmdRotate) < MIN_TURN_POWER) break;

            double tFeed = feederTimer.seconds();
            if (!feederStarted && tFeed >= feederDelaySec) feederStarted = true;

            if (feederStarted && tFeed <= feederDelaySec + feederRunSec) {
                feederMotor.setPower(FEEDER_POWER);
            } else {
                feederMotor.setPower(0.0);
            }

            mecanumDrive(cmdForward, 0.0, cmdRotate);

            telemetry.addLine("DRIVE + FEED (PID w/ decel)");
            telemetry.addData("errX(mm)", "%.1f", errorX);
            telemetry.addData("absErrX(mm)", "%.1f", absErrX);
            telemetry.addData("effMaxPower", "%.2f", effMaxPower);
            telemetry.addData("cmdForward", "%.3f", cmdForward);
            telemetry.addData("cmdRotate", "%.3f", cmdRotate);
            telemetry.addData("feederOn", feederStarted && tFeed <= feederDelaySec + feederRunSec);
            telemetry.addData("X/Y", "%.1f / %.1f", xMm, yMm);
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

    private static double stepToward(double cur, double target, double maxDelta) {
        double d = target - cur;
        if (Math.abs(d) <= maxDelta) return target;
        return cur + Math.signum(d) * maxDelta;
    }
}
