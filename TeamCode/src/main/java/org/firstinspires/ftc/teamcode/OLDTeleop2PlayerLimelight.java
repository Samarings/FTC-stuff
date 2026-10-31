package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "OLDTeleop2PlayerLimelight", group = "TeleOp")
public class OLDTeleop2PlayerLimelight extends LinearOpMode {

    // ===== DRIVE MOTORS =====
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    // ===== MECH MOTORS =====
    private DcMotor   intakeMotor;
    private DcMotor   feederMotor;
    private DcMotorEx shooterMotor;

    // ===== PINPOINT =====
    private GoBildaPinpointDriver pinpoint;
    private Pose2D robotPose;

    // ===== LIMELIGHT 3A =====
    private Limelight3A limelight;
    private boolean limelightHasTarget      = false;
    private double  limelightTxDeg          = 0.0;   // FILTERED tx
    private double  limelightTyDeg          = 0.0;   // FILTERED ty
    private double  limelightDistanceMeters = 0.0;   // FILTERED distance
    private int     limelightTagId          = -1;

    // Raw values (for telemetry/debug)
    private double limelightTxRawDeg = 0.0;
    private double limelightTyRawDeg = 0.0;
    private double limelightDistRawM = 0.0;

    // === ORIGINAL (RIGHT-SIDE-UP) LIMELIGHT GEOMETRY ===
    private static final double LIMELIGHT_HEIGHT_M  = 0.50;
    private static final double TARGET_HEIGHT_M     = 0.90;
    private static final double LIMELIGHT_PITCH_DEG = 20.0;

    // =========================================================
    //          STRICTER LIMELIGHT LOCK SETTINGS
    // =========================================================
    private static final double LL_EMA_ALPHA = 0.35;
    private static final double TAG_AIM_DEADBAND_DEG = 0.50;
    private static final int TAG_LOCK_STABLE_CYCLES = 5;
    private static final boolean LIMELIGHT_LOCK_TO_TAG_ID = true;

    private boolean llFilterInitialized = false;
    private int tagLockStableCount = 0;
    private int lockedTagIdWhenA = -1;
    private boolean g1APrev = false;

    // Limelight auto-aim gains
    private static final double TAG_TURN_KP              = 0.02;
    private static final double TAG_MAX_TURN_POWER       = 0.8;
    private static final double TAG_MAX_TURN_POWER_FINE  = 0.35;

    // =========================================================
    //          SNAPPING (QUICK + PRECISE)
    // =========================================================
    private enum SnapMode { NONE, QUICK, PRECISE }
    private SnapMode snapMode = SnapMode.NONE;
    private double snapTargetDeg = 0.0;
    private double snapEndTimeSec = 0.0;

    // QUICK SNAP: fast, “good enough”, but MUST reach target (no mid-stop)
    private static final double QUICK_SNAP_TIMEOUT_SEC = 1.00; // was ~0.22, caused mid-stop
    private static final double QUICK_SNAP_KP          = 0.10;
    private static final double QUICK_SNAP_MAX_POWER   = 1.00;
    private static final double QUICK_SNAP_MIN_POWER   = 0.35;
    private static final double QUICK_SNAP_STOP_DEG    = 6.0;  // stop once "close enough"

    // PRECISE SNAP (for B=0): accurate, no stall after snapped
    private static final double PRECISE_SNAP_TIMEOUT_SEC = 1.75;
    private static final double PRECISE_SNAP_KP          = 0.035;
    private static final double PRECISE_SNAP_MAX_POWER   = 0.80;
    private static final double PRECISE_SNAP_MAX_FINE    = 0.25;
    private static final double PRECISE_SNAP_FINE_ZONE_DEG = 12.0;
    private static final double PRECISE_SNAP_STOP_DEG    = 0.8;

    // trigger/bumper edge tracking
    private boolean g1RTPrev = false;
    private boolean g1LTPrev = false;
    private boolean g1RBPrev = false;
    private boolean g1LBPrev = false;
    private boolean g1BPrev  = false;

    // =========================================================
    //          SNAP-DRIVE MODE (G1 LS BUTTON TOGGLE)
    // =========================================================
    private boolean snapDriveMode = false; // OFF at startup
    private boolean g1LSPrev = false;

    private static final double SNAP_DRIVE_STICK_DEADBAND = 0.15;
    private static final double SNAP_DRIVE_KP             = 0.040; // accurate
    private static final double SNAP_DRIVE_MAX_POWER      = 0.85;
    private static final double SNAP_DRIVE_DEADBAND_DEG   = 1.0;
    private static final double SNAP_DRIVE_MANUAL_OVERRIDE= 0.12; // right stick overrides auto-rotate

    // ===== SHOOTER CONSTANTS =====
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double MIN_AUTO_RPM = 2850.0;
    private static final double MAX_RPM      = 6000.0;

    // ===== SHOOTER PIDF =====
    private static final double SHOOTER_P_NEAR = 9.3;
    private static final double SHOOTER_I_NEAR = 0.0;
    private static final double SHOOTER_D_NEAR = 0.4;
    private static final double SHOOTER_F_NEAR = 13.65;

    private static final double SHOOTER_P_FAR = 8.1;
    private static final double SHOOTER_I_FAR = 0.0;
    private static final double SHOOTER_D_FAR = 0.3;
    private static final double SHOOTER_F_FAR = 13.65;

    private boolean shooterPidIsFar = false;

    // Manual RPM target (autoRPM off)
    private double shooterRpm = 3000.0;
    private static final double SHOOTER_RPM_STEP = 250.0;

    // ===== AUTO-RPM sqrt(distance) model =====
    private static final double AUTO_RPM_REF_DIST_M = 1.0;
    private static final double AUTO_RPM_REF_RPM    = 3100.0;

    private double shooterRpmSqrtGain = 828.0;
    private static final double SHOOTER_RPM_SQRT_GAIN_STEP = 25.0;

    private boolean autoRpmEnabled = false;
    private boolean g2APrev        = false;

    // Intake / feeder powers
    private static final double INTAKE_FWD_POWER = 1.0;
    private static final double INTAKE_REV_POWER = 1.0;
    private static final double FEEDER_FWD_POWER = 0.85;
    private static final double FEEDER_REV_POWER = 1.0;

    // Shooter toggle + dpad edges (GAMEPAD 2)
    private boolean shooterOn       = false;
    private boolean g2BPrev         = false;
    private boolean g2DpadUpPrev    = false;
    private boolean g2DpadDnPrev    = false;
    private boolean g2DpadLeftPrev  = false;
    private boolean g2DpadRightPrev = false;
    private boolean g2YPrev         = false;

    // G2.X reverse override
    private boolean g2XPrev         = false;
    private static final double SHOOTER_REVERSE_POWER = -1.0;

    // Feeder kickback
    private boolean feederKickActive    = false;
    private double  feederKickEndTime   = 0.0;
    private static final double FEEDER_KICK_DURATION = 0.40;
    private static final double FEEDER_KICK_POWER    = -0.6;

    // Shooter ramp
    private double shooterRpmCmd      = 0.0;
    private double lastLoopTimeSec    = 0.0;

    private static final double SHOOTER_RAMP_UP_RPM_PER_SEC_NEAR   = 2500.0;
    private static final double SHOOTER_RAMP_UP_RPM_PER_SEC_FAR    = 3500.0;
    private static final double SHOOTER_RAMP_DOWN_RPM_PER_SEC_NEAR = 3500.0;
    private static final double SHOOTER_RAMP_DOWN_RPM_PER_SEC_FAR  = 3500.0;

    // Driver turn sensitivity (G1 dpad)
    private double turnSensitivity = 0.7;
    private static final double TURN_SENS_MIN  = 0.2;
    private static final double TURN_SENS_MAX  = 1.0;
    private static final double TURN_SENS_STEP = 0.1;
    private boolean g1DpadUpPrev   = false;
    private boolean g1DpadDnPrev   = false;

    // Driver speed toggle (G1 Y)
    private static final double DRIVE_SPEED_HIGH = 1.0;
    private static final double DRIVE_SPEED_LOW  = 0.4;
    private boolean slowMode  = false;
    private boolean g1YPrev   = false;

    // Tag bias (far shots)
    private boolean tagBiasEnabled = true;
    private static final double FAR_DISTANCE_THRESHOLD_M = 1.75;
    private static final double TAG_AIM_BIAS_DEG         = 2.0;

    // FAR AutoRPM constant RPM
    private static final double FAR_AUTO_RPM = 3990.0;

    // =========================================================
    //              HARSH SHOOTER BRAKE KICK
    // =========================================================
    private boolean shooterBrakeActive = false;
    private double  shooterBrakeEndTime = 0.0;
    private int     shooterBrakeInitialSign = 0;

    private static final double SHOOTER_BRAKE_MIN_POWER = 0.8;
    private static final double SHOOTER_BRAKE_MAX_POWER = 1.00;
    private static final double SHOOTER_BRAKE_MIN_TIME  = 0.16;
    private static final double SHOOTER_BRAKE_MAX_TIME  = 1.5;
    private static final double SHOOTER_BRAKE_RPM_FULL  = 4500.0;
    private static final double SHOOTER_BRAKE_STOP_RPM  = 100;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class,  "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,  "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class,  "backLeft");
        backRight  = hardwareMap.get(DcMotor.class,  "backRight");

        intakeMotor  = hardwareMap.get(DcMotor.class,   "intake");
        feederMotor  = hardwareMap.get(DcMotor.class,   "feeder");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

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
        applyShooterPIDF(false);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Teleop2PlayerLimelight: Quick snaps fixed + B=precise 0 snap");
        telemetry.update();

        waitForStart();
        lastLoopTimeSec = getRuntime();

        while (opModeIsActive()) {

            double now = getRuntime();
            double dt = now - lastLoopTimeSec;
            if (dt <= 0) dt = 0.02;
            if (dt > 0.2) dt = 0.02;
            lastLoopTimeSec = now;

            pinpoint.update();
            robotPose = pinpoint.getPosition();

            double headingRad = robotPose.getHeading(AngleUnit.RADIANS);
            double headingDeg = robotPose.getHeading(AngleUnit.DEGREES);

            updateLimelightOriginalFiltered();

            // SnapDrive toggle (G1 left stick button)
            if (gamepad1.left_stick_button && !g1LSPrev) {
                snapDriveMode = !snapDriveMode;
            }
            g1LSPrev = gamepad1.left_stick_button;

            // G1.A rising edge: capture locked tag id
            if (gamepad1.a && !g1APrev) {
                lockedTagIdWhenA = limelightTagId;
                tagLockStableCount = 0;
            }
            if (!gamepad1.a) {
                lockedTagIdWhenA = -1;
                tagLockStableCount = 0;
            }
            g1APrev = gamepad1.a;

            // Driver turn sensitivity (G1 dpad)
            if (gamepad1.dpad_up && !g1DpadUpPrev) turnSensitivity += TURN_SENS_STEP;
            if (gamepad1.dpad_down && !g1DpadDnPrev) turnSensitivity -= TURN_SENS_STEP;
            g1DpadUpPrev = gamepad1.dpad_up;
            g1DpadDnPrev = gamepad1.dpad_down;
            turnSensitivity = Range.clip(turnSensitivity, TURN_SENS_MIN, TURN_SENS_MAX);

            // Driver speed toggle (G1 Y)
            if (gamepad1.y && !g1YPrev) slowMode = !slowMode;
            g1YPrev = gamepad1.y;
            double driveSpeedScale = slowMode ? DRIVE_SPEED_LOW : DRIVE_SPEED_HIGH;

            // ===== SNAP bindings (CLICK to start, then it completes) =====
            boolean rtSnap = gamepad1.right_trigger > 0.5; // RT = -45
            boolean ltSnap = gamepad1.left_trigger  > 0.5; // LT = +45
            boolean rbSnap = gamepad1.right_bumper;        // RB = -135
            boolean lbSnap = gamepad1.left_bumper;         // LB = +135

            // one-shot (rising edge)
            if (rtSnap && !g1RTPrev) startQuickSnap(now, -45.0);
            if (ltSnap && !g1LTPrev) startQuickSnap(now,  45.0);
            if (rbSnap && !g1RBPrev) startQuickSnap(now, -135.0);
            if (lbSnap && !g1LBPrev) startQuickSnap(now,  135.0);

            // B -> PRECISE 0 snap
            if (gamepad1.b && !g1BPrev) startPreciseSnap(now, 0.0);

            g1RTPrev = rtSnap;
            g1LTPrev = ltSnap;
            g1RBPrev = rbSnap;
            g1LBPrev = lbSnap;
            g1BPrev  = gamepad1.b;

            // Driver translation (field-centric)
            double fieldY = -gamepad1.left_stick_y;
            double fieldX =  gamepad1.left_stick_x;

            fieldY *= driveSpeedScale;
            fieldX *= driveSpeedScale;

            double rightStick = gamepad1.right_stick_x;
            double rotate = rightStick * turnSensitivity;

            // ROTATE priority:
            // 1) Snap (quick/precise)
            // 2) Limelight auto-aim (G1.A hold)
            // 3) SnapDrive mode auto-heading
            // 4) Manual rotate
            if (snapMode != SnapMode.NONE) {
                rotate = updateSnapRotate(now, headingDeg);
            }
            else if (gamepad1.a) {
                boolean canAim = limelightHasTarget;
                if (canAim && LIMELIGHT_LOCK_TO_TAG_ID && lockedTagIdWhenA != -1 && limelightTagId != -1) {
                    if (limelightTagId != lockedTagIdWhenA) canAim = false;
                }

                if (canAim) {
                    double aimOffsetDeg = 0.0;
                    if (tagBiasEnabled &&
                            limelightDistanceMeters > FAR_DISTANCE_THRESHOLD_M &&
                            limelightTagId != -1) {

                        if (limelightTagId == 20) aimOffsetDeg = TAG_AIM_BIAS_DEG;
                        else if (limelightTagId == 24) aimOffsetDeg = -TAG_AIM_BIAS_DEG;
                    }

                    double bearingDeg = limelightTxDeg - aimOffsetDeg;

                    if (Math.abs(bearingDeg) <= TAG_AIM_DEADBAND_DEG) tagLockStableCount++;
                    else tagLockStableCount = 0;

                    if (tagLockStableCount >= TAG_LOCK_STABLE_CYCLES) {
                        rotate = 0.0;
                    } else {
                        double maxTurn = (Math.abs(bearingDeg) <= (TAG_AIM_DEADBAND_DEG * 2.0))
                                ? TAG_MAX_TURN_POWER_FINE : TAG_MAX_TURN_POWER;
                        rotate = Range.clip(TAG_TURN_KP * bearingDeg, -maxTurn, maxTurn);
                    }
                } else {
                    tagLockStableCount = 0;
                    rotate = 0.0;
                }
            }
            else if (snapDriveMode) {
                double mag = Math.hypot(fieldX, fieldY);
                if (mag > SNAP_DRIVE_STICK_DEADBAND && Math.abs(rightStick) < SNAP_DRIVE_MANUAL_OVERRIDE) {
                    double travelDeg = Math.toDegrees(Math.atan2(fieldX, fieldY));
                    double targetDeg = pickClosestSideHeading(headingDeg, travelDeg);

                    double errDeg = wrapAngleDeg(headingDeg - targetDeg);
                    if (Math.abs(errDeg) < SNAP_DRIVE_DEADBAND_DEG) {
                        rotate = 0.0;
                    } else {
                        rotate = Range.clip(SNAP_DRIVE_KP * errDeg, -SNAP_DRIVE_MAX_POWER, SNAP_DRIVE_MAX_POWER);
                    }
                }
            }

            // field-centric transform
            double rotX = fieldX * Math.cos(-headingRad) - fieldY * Math.sin(-headingRad);
            double rotY = fieldX * Math.sin(-headingRad) + fieldY * Math.cos(-headingRad);

            mecanumDrive(rotY, rotX, rotate);

            // ===== Operator controls (G2) =====

            // Shooter toggle (G2.B) + kickback window + brake on OFF
            if (gamepad2.b && !g2BPrev) {
                boolean wasOn = shooterOn;
                shooterOn = !shooterOn;

                if (shooterOn) {
                    shooterBrakeActive = false;
                    feederKickActive = true;
                    feederKickEndTime = now + FEEDER_KICK_DURATION;
                } else if (wasOn) {
                    startShooterBrake(now);
                }
            }
            g2BPrev = gamepad2.b;

            // AutoRPM toggle (G2.A)
            if (gamepad2.a && !g2APrev) autoRpmEnabled = !autoRpmEnabled;
            g2APrev = gamepad2.a;

            // Manual RPM (G2 dpad up/down) when autoRPM OFF
            if (!autoRpmEnabled) {
                if (gamepad2.dpad_up && !g2DpadUpPrev) shooterRpm += SHOOTER_RPM_STEP;
                if (gamepad2.dpad_down && !g2DpadDnPrev) shooterRpm -= SHOOTER_RPM_STEP;
            }
            g2DpadUpPrev = gamepad2.dpad_up;
            g2DpadDnPrev = gamepad2.dpad_down;

            // Tune ONE constant for autoRPM (G2 dpad left/right)
            if (gamepad2.dpad_right && !g2DpadRightPrev) shooterRpmSqrtGain += SHOOTER_RPM_SQRT_GAIN_STEP;
            if (gamepad2.dpad_left  && !g2DpadLeftPrev)  shooterRpmSqrtGain -= SHOOTER_RPM_SQRT_GAIN_STEP;
            g2DpadRightPrev = gamepad2.dpad_right;
            g2DpadLeftPrev  = gamepad2.dpad_left;

            shooterRpmSqrtGain = Range.clip(shooterRpmSqrtGain, 0.0, 5000.0);

            // AUTO-RPM (WITH FAR OVERRIDE)
            boolean farAutoMode = false;
            if (autoRpmEnabled && limelightHasTarget) {
                double d = Math.max(0.0, limelightDistanceMeters);
                if (d > FAR_DISTANCE_THRESHOLD_M) {
                    shooterRpm = FAR_AUTO_RPM;
                    farAutoMode = true;
                } else {
                    double autoRpm = AUTO_RPM_REF_RPM
                            + shooterRpmSqrtGain * (Math.sqrt(d) - Math.sqrt(AUTO_RPM_REF_DIST_M));
                    shooterRpm = Range.clip(autoRpm, MIN_AUTO_RPM, MAX_RPM);
                    farAutoMode = false;
                }
            }

            shooterRpm = Range.clip(shooterRpm, 0.0, MAX_RPM);

            boolean wantFarPid = autoRpmEnabled && limelightHasTarget && farAutoMode;
            if (wantFarPid != shooterPidIsFar) {
                applyShooterPIDF(wantFarPid);
                shooterPidIsFar = wantFarPid;
            }

            // Intake
            double intakePower = 0.0;
            if (gamepad2.left_trigger > 0.05) intakePower = gamepad2.left_trigger * INTAKE_FWD_POWER;
            if (gamepad2.left_bumper) intakePower = -INTAKE_REV_POWER;
            if (gamepad2.y && !g2YPrev) intakePower = INTAKE_FWD_POWER * 0.1;
            g2YPrev = gamepad2.y;
            intakeMotor.setPower(intakePower);

            // Feeder
            double feederPower = 0.0;
            if (shooterOn && feederKickActive && now < feederKickEndTime) {
                feederPower = FEEDER_KICK_POWER;
            } else {
                feederKickActive = false;
                if (gamepad2.right_trigger > 0.05) feederPower = gamepad2.right_trigger * FEEDER_FWD_POWER;
                if (gamepad2.right_bumper) feederPower = -FEEDER_REV_POWER;
            }
            feederMotor.setPower(feederPower);

            // G2.X reverse override (held)
            boolean reverseOverride = gamepad2.x;
            if (!reverseOverride && g2XPrev && !shooterOn) {
                startShooterBrake(now);
            }
            g2XPrev = reverseOverride;

            // Shooter command
            if (reverseOverride) {
                shooterBrakeActive = false;
                shooterRpmCmd = 0.0;

                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterMotor.setPower(SHOOTER_REVERSE_POWER);
            }
            else if (shooterBrakeActive) {
                updateShooterBrake(now);
                shooterRpmCmd = 0.0;
            }
            else {
                double targetRpm = shooterOn ? shooterRpm : 0.0;

                double rampUp   = farAutoMode ? SHOOTER_RAMP_UP_RPM_PER_SEC_FAR   : SHOOTER_RAMP_UP_RPM_PER_SEC_NEAR;
                double rampDown = farAutoMode ? SHOOTER_RAMP_DOWN_RPM_PER_SEC_FAR : SHOOTER_RAMP_DOWN_RPM_PER_SEC_NEAR;

                double maxDelta = ((targetRpm > shooterRpmCmd) ? rampUp : rampDown) * dt;
                shooterRpmCmd = stepToward(shooterRpmCmd, targetRpm, maxDelta);

                if (shooterRpmCmd > 1.0) {
                    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    double ticksPerSecCmd = shooterRpmCmd * SHOOTER_TICKS_PER_REV / 60.0;
                    shooterMotor.setVelocity(ticksPerSecCmd);
                } else {
                    shooterMotor.setPower(0.0);
                }
            }
// ===== Telemetry (restored like original) =====
            double xMm = robotPose.getX(DistanceUnit.MM);
            double yMm = robotPose.getY(DistanceUnit.MM);

// Shooter measured RPM
            double shooterRpmMeas = getShooterRpmMeasured();

            telemetry.addLine("Drive (G1)");
            telemetry.addData("Speed", slowMode ? "LOW" : "HIGH");
            telemetry.addData("TurnSens", "%.2f", turnSensitivity);
            telemetry.addData("Pose x/y (mm)", "%.0f / %.0f", xMm, yMm);
            telemetry.addData("Heading (deg)", "%.1f", headingDeg);

// Snap info (new)
            telemetry.addData("SnapMode", snapMode);
            telemetry.addData("SnapDriveMode (LS)", snapDriveMode);

            telemetry.addLine("\nShooter/Feed (G2)");
            telemetry.addData("ShooterOn (B)", shooterOn);
            telemetry.addData("ReverseOverride (X)", reverseOverride);
            telemetry.addData("BrakeActive", shooterBrakeActive);
            telemetry.addData("AutoRPM (A)", autoRpmEnabled);
            telemetry.addData("FAR auto mode", farAutoMode);
            telemetry.addData("RPM target/cmd/meas", "%.0f / %.0f / %.0f", shooterRpm, shooterRpmCmd, shooterRpmMeas);
            telemetry.addData("AutoGain (RPM/√m)", "%.1f", shooterRpmSqrtGain);
            telemetry.addData("RampUp near/far", "%.0f / %.0f", SHOOTER_RAMP_UP_RPM_PER_SEC_NEAR, SHOOTER_RAMP_UP_RPM_PER_SEC_FAR);
            telemetry.addData("FeederPower", "%.2f", feederPower);
            telemetry.addData("IntakePower", "%.2f", intakePower);

            telemetry.addLine("\nShooter PIDF (active)");
            if (shooterPidIsFar) {
                telemetry.addData("MODE", "FAR");
                telemetry.addData("P I D F", "%.2f %.2f %.2f %.2f", SHOOTER_P_FAR, SHOOTER_I_FAR, SHOOTER_D_FAR, SHOOTER_F_FAR);
            } else {
                telemetry.addData("MODE", "NEAR");
                telemetry.addData("P I D F", "%.2f %.2f %.2f %.2f", SHOOTER_P_NEAR, SHOOTER_I_NEAR, SHOOTER_D_NEAR, SHOOTER_F_NEAR);
            }

            telemetry.addLine("\nLimelight (ORIGINAL + FILTER)");
            telemetry.addData("HasTarget", limelightHasTarget);
            telemetry.addData("TagId (cur/locked)", "%d / %d", limelightTagId, lockedTagIdWhenA);
            telemetry.addData("tx raw/filt", "%.2f / %.2f", limelightTxRawDeg, limelightTxDeg);
            telemetry.addData("ty raw/filt", "%.2f / %.2f", limelightTyRawDeg, limelightTyDeg);
            telemetry.addData("dist raw/filt (m)", "%.2f / %.2f", limelightDistRawM, limelightDistanceMeters);
            //telemetry.addData("stableCount", "%d/%d", tagLockStableCount, TAG_LOCK_STABLE_CYCLES);
            //telemetry.addData("deadband", "%.2f", TAG_AIM_DEADBAND_DEG);

            telemetry.update();

        }

        if (limelight != null) limelight.stop();
    }

    // ===== SNAP HELPERS =====
    private void startQuickSnap(double nowSec, double targetDeg) {
        snapMode = SnapMode.QUICK;
        snapTargetDeg = wrapAngleDeg(targetDeg);
        snapEndTimeSec = nowSec + QUICK_SNAP_TIMEOUT_SEC;
    }

    private void startPreciseSnap(double nowSec, double targetDeg) {
        snapMode = SnapMode.PRECISE;
        snapTargetDeg = wrapAngleDeg(targetDeg);
        snapEndTimeSec = nowSec + PRECISE_SNAP_TIMEOUT_SEC;
    }

    private double updateSnapRotate(double nowSec, double headingDeg) {
        if (snapMode == SnapMode.NONE) return 0.0;

        if (nowSec >= snapEndTimeSec) {
            snapMode = SnapMode.NONE;
            return 0.0;
        }

        double errDeg = wrapAngleDeg(headingDeg - snapTargetDeg);

        if (snapMode == SnapMode.QUICK) {
            if (Math.abs(errDeg) <= QUICK_SNAP_STOP_DEG) {
                snapMode = SnapMode.NONE;
                return 0.0;
            }

            double cmd = QUICK_SNAP_KP * errDeg;
            cmd = Range.clip(cmd, -QUICK_SNAP_MAX_POWER, QUICK_SNAP_MAX_POWER);

            if (Math.abs(cmd) < QUICK_SNAP_MIN_POWER) cmd = Math.signum(cmd) * QUICK_SNAP_MIN_POWER;
            return cmd;
        }

        // PRECISE
        if (Math.abs(errDeg) <= PRECISE_SNAP_STOP_DEG) {
            snapMode = SnapMode.NONE;
            return 0.0;
        }

        double maxTurn = (Math.abs(errDeg) <= PRECISE_SNAP_FINE_ZONE_DEG)
                ? PRECISE_SNAP_MAX_FINE
                : PRECISE_SNAP_MAX_POWER;

        double cmd = PRECISE_SNAP_KP * errDeg;
        return Range.clip(cmd, -maxTurn, maxTurn);
    }

    // pick closest robot side to travel direction
    private double pickClosestSideHeading(double currentHeadingDeg, double travelDeg) {
        double[] candidates = new double[] {
                travelDeg,
                travelDeg + 180.0,
                travelDeg + 90.0,
                travelDeg - 90.0
        };

        double best = wrapAngleDeg(candidates[0]);
        double bestAbs = Math.abs(wrapAngleDeg(currentHeadingDeg - best));

        for (int i = 1; i < candidates.length; i++) {
            double c = wrapAngleDeg(candidates[i]);
            double a = Math.abs(wrapAngleDeg(currentHeadingDeg - c));
            if (a < bestAbs) {
                bestAbs = a;
                best = c;
            }
        }
        return best;
    }

    // ===== LIMELIGHT FILTER =====
    private void updateLimelightOriginalFiltered() {
        limelightHasTarget      = false;
        limelightTagId          = -1;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            llFilterInitialized = false;
            limelightTxDeg = limelightTyDeg = limelightDistanceMeters = 0.0;
            limelightTxRawDeg = limelightTyRawDeg = limelightDistRawM = 0.0;
            return;
        }

        limelightHasTarget = true;

        limelightTxRawDeg = result.getTx();
        limelightTyRawDeg = result.getTy();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            limelightTagId = fiducials.get(0).getFiducialId();
        }

        double totalAngleDeg = LIMELIGHT_PITCH_DEG + limelightTyRawDeg;
        double totalAngleRad = Math.toRadians(totalAngleDeg);
        double heightDiff    = TARGET_HEIGHT_M - LIMELIGHT_HEIGHT_M;

        double tan = Math.tan(totalAngleRad);
        if (tan > 1e-3) {
            limelightDistRawM = Range.clip(heightDiff / tan, 0.0, 10.0);
        } else {
            limelightDistRawM = 0.0;
        }

        if (!llFilterInitialized) {
            limelightTxDeg = limelightTxRawDeg;
            limelightTyDeg = limelightTyRawDeg;
            limelightDistanceMeters = limelightDistRawM;
            llFilterInitialized = true;
        } else {
            limelightTxDeg = LL_EMA_ALPHA * limelightTxRawDeg + (1.0 - LL_EMA_ALPHA) * limelightTxDeg;
            limelightTyDeg = LL_EMA_ALPHA * limelightTyRawDeg + (1.0 - LL_EMA_ALPHA) * limelightTyDeg;
            limelightDistanceMeters = LL_EMA_ALPHA * limelightDistRawM + (1.0 - LL_EMA_ALPHA) * limelightDistanceMeters;
        }
    }

    // ===== APPLY PIDF =====
    private void applyShooterPIDF(boolean far) {
        try {
            if (far) {
                shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P_FAR, SHOOTER_I_FAR, SHOOTER_D_FAR, SHOOTER_F_FAR);
            } else {
                shooterMotor.setVelocityPIDFCoefficients(SHOOTER_P_NEAR, SHOOTER_I_NEAR, SHOOTER_D_NEAR, SHOOTER_F_NEAR);
            }
        } catch (Exception ignored) { }
    }

    // ===== dynamic harsh brake kick =====
    private void startShooterBrake(double nowSec) {
        double rpm = getShooterRpmMeasured();
        double rpmAbs = Math.abs(rpm);

        if (rpmAbs < SHOOTER_BRAKE_STOP_RPM) {
            shooterBrakeActive = false;
            shooterMotor.setPower(0.0);
            return;
        }

        double s = Range.clip(rpmAbs / SHOOTER_BRAKE_RPM_FULL, 0.0, 1.0);

        double brakeTime = SHOOTER_BRAKE_MIN_TIME + (SHOOTER_BRAKE_MAX_TIME - SHOOTER_BRAKE_MIN_TIME) * s;
        shooterBrakeEndTime = nowSec + brakeTime;

        shooterBrakeInitialSign = (rpm >= 0) ? 1 : -1;
        shooterBrakeActive = true;

        updateShooterBrake(nowSec);
    }

    private void updateShooterBrake(double nowSec) {
        if (nowSec >= shooterBrakeEndTime) {
            stopShooterBrake();
            return;
        }

        double rpm = getShooterRpmMeasured();
        double rpmAbs = Math.abs(rpm);

        if (rpmAbs < SHOOTER_BRAKE_STOP_RPM) {
            stopShooterBrake();
            return;
        }

        int signNow = (rpm >= 0) ? 1 : -1;
        if (signNow != shooterBrakeInitialSign) {
            stopShooterBrake();
            return;
        }

        double s = Range.clip(rpmAbs / SHOOTER_BRAKE_RPM_FULL, 0.0, 1.0);
        double brakePowerMag = SHOOTER_BRAKE_MIN_POWER + (SHOOTER_BRAKE_MAX_POWER - SHOOTER_BRAKE_MIN_POWER) * s;
        brakePowerMag = Range.clip(brakePowerMag, 0.0, 1.0);

        double brakePower = -Math.signum(rpm) * brakePowerMag;

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(brakePower);
    }

    private void stopShooterBrake() {
        shooterBrakeActive = false;
        shooterMotor.setPower(0.0);
    }

    private double getShooterRpmMeasured() {
        double ticksPerSec = shooterMotor.getVelocity();
        return ticksPerSec * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    // mecanumDrive
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
