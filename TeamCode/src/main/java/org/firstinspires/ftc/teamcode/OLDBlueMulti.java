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

@Autonomous(name = "OLDbluemulti", group = "Auto")
public class OLDBlueMulti extends LinearOpMode {

    // ===== DRIVE MOTORS =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
    // ===== SHOOTER KICK-BRAKE (reverse pulse at end of shooting) =====
    private static final double SHOOTER_BRAKE_POWER    = 1;  // 0.15..0.30 typical
    private static final double SHOOTER_BRAKE_TIME_SEC = 0.4;  // 0.05..0.12 sec typical
    private static final double SHOOTER_BRAKE_MIN_RPM  = 100.0; // only brake if spinning

    // ===== MECHANISM MOTORS =====
    private DcMotor   intakeMotor;
    private DcMotor   feederMotor;
    private DcMotorEx shooterMotor;
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

    // Distance estimate constants – TUNE THESE FOR YOUR MOUNT
    private static final double LIMELIGHT_HEIGHT_M  = 0.50;  // camera center height
    private static final double TARGET_HEIGHT_M     = 0.90;  // target center height
    private static final double LIMELIGHT_PITCH_DEG = 20.0;  // up-tilt from horizontal

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

    // ===== SHOOTER / INTAKE CONSTANTS =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0;     // goBILDA 5203 series
    private static final double AUTO_SHOOTER_RPM      = 3200;   // your current auto target (tune)

    private static final double INTAKE_POWER      = 1.0;
    private static final double INTAKE_LOW_POWER  = 0.75;
    private static final double FEEDER_POWER      = 0.6;

    // ===== FEEDER KICKBACK (pre-shot) =====
    private static final double FEEDER_KICKBACK_TIME_SEC     = 0.3;
    private static final double FEEDER_KICKBACK_POWER_MAG    = 1.0;
    private static final double FEEDER_KICKBACK_SETTLE_SEC   = 0.00;

    // ===== SHOOTER PIDF (REV velocity loop) =====
    // goBILDA 6000RPM: max ticks/sec ~= 6000/60*28 = 2800 => F ~ 32767/2800 ~ 11.7
    // You were using these in TeleOp; keep consistent here.
    private static final double SHOOTER_P = 9.3;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.4;
    private static final double SHOOTER_F = 13.65;

    // ===== SHOOTER RAMP (RPM setpoint ramp) =====
    private double shooterRpmCmd = 0.0;
    private static final double SHOOTER_RAMP_UP_RPM_PER_SEC   = 2700.0;
    private static final double SHOOTER_RAMP_DOWN_RPM_PER_SEC = 9900.0;

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

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== APPLY SHOOTER PIDF =====
        applyShooterPIDF();

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

        telemetry.addLine("BlueMulti READY (Pinpoint + Limelight + Shooter PIDF + Shooter ramp)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ===== AUTO SEQUENCE =====
        intakeOnLow();
        driveBackward(575, 1.0, 4);
        strafeLeft(750, 0.8, 3.0);
        intakeOff();
        turnToHeading(-40.0, 0.35, 2.0);

        double distanceM = limelightLock(0.8);

        telemetry.addLine("After Limelight lock");
        telemetry.addData("Estimated distance (m)", "%.3f", distanceM);
        telemetry.addData("Shooter RPM target", "%.1f", AUTO_SHOOTER_RPM);
        telemetry.update();

        // shoot(spinupSec, shootSec, feederPow, intakePow)
        shoot(2.0, 2.2, 1.0, 0.80);

        turnToHeading(1, 0.3, 1.2);
        strafeLeft(340, 1.0, 3);

        intakeOnForward();
        driveFeed(0.60, 900, 0.4, 0.15,450,0.2);
        turnToHeading(37.0, 0.35, 1.2);
        intakeOnForward();
        driveBackward(470, 1, 3.0);

        turnToHeading(-43.0, 0.35, 1.2);
        distanceM = limelightLock(0.8);
        intakeOff();
        telemetry.addLine("After Limelight lock");
        telemetry.addData("Estimated distance (m)", "%.3f", distanceM);
        telemetry.addData("Shooter RPM target", "%.1f", AUTO_SHOOTER_RPM);
        telemetry.update();

        shoot(2.0, 2.0, 1.00, 0.80);
        turnToHeading(1, 0.3, 1.2);
        strafeLeft(1000, 1.0, 3);

        intakeOnForward();
        driveFeed(0.60, 600, 0.4, 0.15,450,0.25);
        turnToHeading(55, 0.35, 1.2);
        intakeOnForward();
        driveBackward(590, 1, 3.0);

        turnToHeading(-43.0, 0.35, 2.0);
        intakeOff();
        distanceM = limelightLock(0.8);

        telemetry.addLine("After Limelight lock");
        telemetry.addData("Estimated distance (m)", "%.3f", distanceM);
        telemetry.addData("Shooter RPM target", "%.1f", AUTO_SHOOTER_RPM);
        telemetry.update();

        shoot(2.0, 2.2, 1.0, 0.80);
        strafeLeft(350, 1.0, 4);
        telemetry.addLine("AUTO DONE");
        telemetry.update();

        if (limelight != null) limelight.stop();
    }

    // =========================================================
    //                  SHOOTER PIDF + RAMP HELPERS
    // =========================================================
    private void shooterKickBrakePulse() {
        // Measure current speed (signed). If your motor direction is reversed, the sign flips too.
        double velTicksPerSec = shooterMotor.getVelocity();
        double rpmMeas = Math.abs(velTicksPerSec) * 60.0 / SHOOTER_TICKS_PER_REV;

        if (rpmMeas < SHOOTER_BRAKE_MIN_RPM) return; // don’t “kick” when basically stopped

        double sign = (Math.abs(velTicksPerSec) > 1e-6) ? Math.signum(velTicksPerSec) : 1.0;

        // Reverse relative to current spin direction
        double reversePower = -sign * SHOOTER_BRAKE_POWER;
        reversePower = Range.clip(reversePower, -1.0, 1.0);

        // Use open-loop for the short brake pulse (fast + predictable)
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(reversePower);
        sleep((long) (SHOOTER_BRAKE_TIME_SEC * 1000));

        shooterMotor.setPower(0.0);
        // No need to set mode back here; shooterOn() will set RUN_USING_ENCODER before setVelocity().
    }

    private void applyShooterPIDF() {
        try {
            shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        } catch (Exception ignored) {
            // If your SDK/hub rejects this call, it will run with default coefficients.
        }
    }

    private void setShooterVelocityRpm(double rpm) {
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ticksPerSec = rpm * SHOOTER_TICKS_PER_REV / 60.0;
        shooterMotor.setVelocity(ticksPerSec);
    }

    private void rampShooterToward(double targetRpm, double dt) {
        double maxDelta = ((targetRpm > shooterRpmCmd) ? SHOOTER_RAMP_UP_RPM_PER_SEC : SHOOTER_RAMP_DOWN_RPM_PER_SEC) * dt;
        shooterRpmCmd = stepToward(shooterRpmCmd, targetRpm, maxDelta);

        if (shooterRpmCmd < 1.0) {
            shooterMotor.setPower(0.0);
            return;
        }
        setShooterVelocityRpm(shooterRpmCmd);
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

            double bearingDeg = limelightTxDeg;
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
     * Kickback -> ramp shooter to AUTO_SHOOTER_RPM for spinupTimeSec -> feed for maxShootTimeSec.
     */
    private void shoot(double spinupTimeSec,
                       double maxShootTimeSec,
                       double feederPower,
                       double intakePower) {

        feederPower = Range.clip(feederPower, -1.0, 1.0);
        intakePower = Range.clip(intakePower, -1.0, 1.0);

        // 1) intake OFF for kickback
        intakeMotor.setPower(0.0);

        // 2) Kickback BEFORE shooter acceleration
        feederKickbackBeforeShoot(feederPower);

        // 3) Spin up shooter using RAMP + PIDF
        double targetRpm = AUTO_SHOOTER_RPM;
        ElapsedTime spinTimer = new ElapsedTime();
        double lastT = 0.0;

        while (opModeIsActive() && spinTimer.seconds() < spinupTimeSec) {
            double t = spinTimer.seconds();
            double dt = t - lastT;
            if (dt <= 0) dt = 0.02;
            if (dt > 0.2) dt = 0.02;
            lastT = t;

            rampShooterToward(targetRpm, dt);

            double measTps = shooterMotor.getVelocity();
            double measRpm = measTps * 60.0 / SHOOTER_TICKS_PER_REV;

            telemetry.addLine("SHOOT SPINUP (PIDF + ramp)");
            telemetry.addData("target/cmd/meas rpm", "%.0f / %.0f / %.0f", targetRpm, shooterRpmCmd, measRpm);
            telemetry.update();

            idle();
        }

        // 4) Feed while continuing to hold RPM (ramp keeps it stable)
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double lastLoop = 0.0;

        while (opModeIsActive() && timer.seconds() < maxShootTimeSec) {
            double t = timer.seconds();
            double dt = t - lastLoop;
            if (dt <= 0) dt = 0.02;
            if (dt > 0.2) dt = 0.02;
            lastLoop = t;

            rampShooterToward(targetRpm, dt);

            feederMotor.setPower(feederPower);
            intakeMotor.setPower(intakePower);

            telemetry.addLine("SHOOTING (PIDF + ramp hold)");
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

    /**
     * OPTIONAL: pulse intake while feeding (feeder constant), shooter holds RPM with PIDF+ramp.
     */
    private void shootWithPulse(double spinupTimeSec,
                                double maxShootTimeSec,
                                double feederPower,
                                double intakeOnPower,
                                double intakePulseIntervalSec) {

        feederPower   = Range.clip(feederPower, -1.0, 1.0);
        intakeOnPower = Range.clip(intakeOnPower, -1.0, 1.0);

        // ramp spinup
        double targetRpm = AUTO_SHOOTER_RPM;
        ElapsedTime spinTimer = new ElapsedTime();
        double lastT = 0.0;

        while (opModeIsActive() && spinTimer.seconds() < spinupTimeSec) {
            double t = spinTimer.seconds();
            double dt = t - lastT;
            if (dt <= 0) dt = 0.02;
            if (dt > 0.2) dt = 0.02;
            lastT = t;

            rampShooterToward(targetRpm, dt);
            idle();
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        boolean intakeOn = false;
        double lastToggle = 0.0;
        double lastLoop = 0.0;

        while (opModeIsActive() && timer.seconds() < maxShootTimeSec) {
            double t = timer.seconds();
            double dt = t - lastLoop;
            if (dt <= 0) dt = 0.02;
            if (dt > 0.2) dt = 0.02;
            lastLoop = t;

            rampShooterToward(targetRpm, dt);

            if (intakePulseIntervalSec > 0 && (t - lastToggle) >= intakePulseIntervalSec) {
                intakeOn = !intakeOn;
                lastToggle = t;
            }

            feederMotor.setPower(feederPower);
            intakeMotor.setPower(intakeOn ? intakeOnPower : 0.0);

            telemetry.addLine("SHOOT (pulse intake)");
            telemetry.addData("t", "%.2f / %.2f", t, maxShootTimeSec);
            telemetry.update();

            idle();
        }

        intakeMotor.setPower(0.0);
        feederMotor.setPower(0.0);
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

    // ===== driveFeed with defaults (keeps your existing calls working) =====
    private void driveFeed(double speed, double distanceMm,
                           double feederDelaySec, double feederRunSec) {
        driveFeed(speed, distanceMm, feederDelaySec, feederRunSec,
                DECEL_DISTANCE_MM, MIN_DECEL_POWER_FRACTION);
    }

    /**
     * Drive along X while turning on feeder after a delay, for a certain duration.
     * Adds a deceleration phase ONLY in the last decelDistanceMm of travel.
     *
     * @param speed              commanded speed sign gives direction; magnitude is maxPower
     * @param distanceMm         distance to travel (mm)
     * @param feederDelaySec     delay before feeder starts
     * @param feederRunSec       how long feeder runs once started
     * @param decelDistanceMm    decel only happens when remaining distance < this
     * @param minPowerFraction   minimum fraction of maxPower allowed near the end (0..1)
     */
    private void driveFeed(double speed, double distanceMm,
                           double feederDelaySec, double feederRunSec,
                           double decelDistanceMm, double minPowerFraction) {

        double dir = Math.signum(speed);
        if (dir == 0) return;

        double maxPower = Math.abs(speed);
        double distance = Math.abs(distanceMm);

        // Clamp decel params
        decelDistanceMm   = Math.max(0.0, decelDistanceMm);
        minPowerFraction  = Range.clip(minPowerFraction, 0.0, 1.0);

        // Snapshot starting pose
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

            // distance "good enough"
            if (absErrX < POSITION_TOL_MM) break;

            // --- POSITION PID (forward) ---
            posIntegral += errorX * dt;
            posIntegral = Range.clip(posIntegral, -500.0, 500.0);
            double posDerivative = (errorX - posLastError) / dt;
            posLastError = errorX;

            double cmdForward = DRIVE_KP * errorX
                    + DRIVE_KI * posIntegral
                    + DRIVE_KD * posDerivative;

// ===== STEP DECEL: once inside last decelDistanceMm, immediately clamp to minPowerFraction =====
            double effMaxPower = maxPower;
            if (decelDistanceMm > 1e-6 && absErrX < decelDistanceMm) {
                effMaxPower = maxPower * minPowerFraction;  // immediate drop
            }
            cmdForward = Range.clip(cmdForward, -effMaxPower, effMaxPower);


            // --- HEADING PID (hold start heading) ---
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

            // If both commands are tiny, we’re basically there → exit
            if (Math.abs(cmdForward) < MIN_MOVE_POWER && Math.abs(cmdRotate) < MIN_TURN_POWER) {
                break;
            }

            // ---- FEEDER TIMING ----
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
