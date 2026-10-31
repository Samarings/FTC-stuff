package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "MainTeleop", group = "TeleOp")
public class MainTeleop extends LinearOpMode {

    // ===================== DRIVE =====================
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===================== MECHS =====================
    private DcMotor intakeMotor;
    private DcMotor feederMotor;
    private DcMotorEx shooterMotor;

    // ===================== LEDs (goBILDA PWM RGB 3118-0808-0002) =====================
    private Servo ledLeft, ledRight;
    private static final double LED_PWM_MIN_US = 500.0;
    private static final double LED_PWM_MAX_US = 2500.0;

    // Your calibrated values
    private static final double LED_RED_US_BASE    = 1101.0;
    private static final double LED_ORANGE_US_BASE = 1142.0;
    private static final double LED_YELLOW_US_BASE = 1200.0;
    private static final double LED_GREEN_US_BASE  = 1500.0;

    private double ledPulseOffsetUs = 0.0; // optional global trim

    private enum LedColor { RED, ORANGE, YELLOW, GREEN }
    private LedColor ledState = LedColor.ORANGE;

    // ===================== STOPPERS =====================
    private Servo leftStop, rightStop;
    private boolean stoppersClosed = true; // default CLOSED
    private boolean stoppersClosedPrev = true;

    // TODO tune positions
    private static final double LEFT_STOP_ZERO_POS   = 0.50; // closed
    private static final double LEFT_STOP_LEFT_POS   = 0.00; // open
    private static final double RIGHT_STOP_LEFT_POS  = 0.00; // closed
    private static final double RIGHT_STOP_RIGHT_POS = 1.00; // open

    // ===================== PINPOINT =====================
    private GoBildaPinpointDriver pinpoint;
    private Pose2D robotPose;

    // Odometry “goal/origin” is (0,0) — DO NOT change
    private static final double GOAL_X_MM = 0.0;
    private static final double GOAL_Y_MM = 0.0;

    // ===================== LOCK-ON TARGET (ODOM FALLBACK) =====================
    private static final double LOCK_TARGET_OFFSET_X_MM = 250; // tweak here
    private static final double LOCK_TARGET_OFFSET_Y_MM = 70;  // tweak here
    private double lockTargetXmm = GOAL_X_MM;
    private double lockTargetYmm = GOAL_Y_MM;

    // ===================== LIMELIGHT 3A =====================
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

    // ORIGINAL (RIGHT-SIDE-UP) LIMELIGHT GEOMETRY
    private static final double LIMELIGHT_HEIGHT_M  = 0.50;
    private static final double TARGET_HEIGHT_M     = 0.90;
    private static final double LIMELIGHT_PITCH_DEG = 20.0;

    // Filter / lock settings (same algorithm)
    private static final double LL_EMA_ALPHA              = 0.35;
    private static final double TAG_AIM_DEADBAND_DEG      = 0.50;
    private static final int    TAG_LOCK_STABLE_CYCLES    = 5;
    private static final boolean LIMELIGHT_LOCK_TO_TAG_ID = true;

    private boolean llFilterInitialized = false;
    private int tagLockStableCount = 0;
    private int lockedTagIdWhenAim = -1;

    // Limelight auto-aim gains (same)
    private static final double TAG_TURN_KP              = 0.02;
    private static final double TAG_MAX_TURN_POWER       = 0.8;
    private static final double TAG_MAX_TURN_POWER_FINE  = 0.35;

    // Aim bias (far shots) (same)
    private boolean tagBiasEnabled = true;
    private static final double FAR_DISTANCE_THRESHOLD_M = 1.75;
    private static final double TAG_AIM_BIAS_DEG         = 2.0;

    // Distance fusion (mix limelight + odom)
    private static final double DIST_FUSION_LL_WEIGHT = 0.85; // "default to limelight"

    // ===================== SHOOTER =====================
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;

    // NEW: IDLE MODE
    private static final double IDLE_RPM = 1200.0;
    private boolean idleEnabled = true;     // default ON
    private boolean g2BackPrev = false;

    // IMPORTANT CHANGE:
    // Idle does NOT create an "active target RPM" and does NOT affect LEDs.
    // Idle is open-loop feedforward (no PID) with a gentle ramp to reduce brownouts.
    private static final double IDLE_POWER_FUDGE = 0.00;       // tune if idle is too low/high
    private static final double IDLE_POWER_RAMP_PER_SEC = 3.0; // power units/sec for ramping
    private double shooterPowerCmdPrev = 0.0;

    // Manual RPM target (G2) when autoRPM OFF
    private double shooterRpmManual = 3000.0;
    private static final double SHOOTER_RPM_STEP = 250.0;

    // === AUTO-RPM method (Teleop2 style) ===
    private static final double MIN_AUTO_RPM        = 2000;
    private static final double AUTO_RPM_REF_DIST_M = 1.0;
    private static final double AUTO_RPM_REF_RPM    = 2400;
    private double shooterRpmSqrtGain               = 400;
    private static final double SHOOTER_RPM_SQRT_GAIN_STEP = 25.0;
    private static final double FAR_AUTO_RPM        = 3050;

    private boolean autoRpmEnabled = true; // ON by default
    private boolean g2LSPrev = false;

    // Custom PIDF (power control) - used ONLY when shooter is active (shooterOn)
    private double kP = 0.015;
    private double kI = 0.0;
    private double kD = 0.0001;
    private double kF = 0.000422143;

    private static final double INTEGRAL_CLAMP = 5000.0;
    private double shooterIntegral = 0.0;
    private double shooterLastError = 0.0;

    private boolean shooterOn = false;
    private boolean g2BPrev = false;

    // Intake / feeder powers
    private static final double INTAKE_FWD_POWER = 1.0;
    private static final double INTAKE_REV_POWER = 1.0;
    private static final double FEEDER_FWD_POWER = 0.85;
    private static final double FEEDER_REV_POWER = 1.0;

    // ===================== A HOLD (OPEN STOPPERS + MAX FEED/INTAKE) =====================
    private static final double A_HOLD_INTAKE_POWER = 1.0;
    private static final double A_HOLD_FEEDER_POWER = 1.0;

    // Edges for RPM tweaks
    private boolean g2DpadUpPrev    = false;
    private boolean g2DpadDnPrev    = false;
    private boolean g2DpadLeftPrev  = false;
    private boolean g2DpadRightPrev = false;

    // ===================== PULSE FEED/INTAKE (G2.X hold) =====================
    private static final double PULSE_ON_SEC  = 0.20;
    private static final double PULSE_OFF_SEC = 0.20;
    private static final double PULSE_INTAKE_POWER = 1.0;
    private static final double PULSE_FEEDER_POWER = 0.85;

    private boolean g2XPrev = false;
    private double pulseStartTimeSec = 0.0;
    private boolean pulseIsOn = false;

    // ===================== DRIVER CONTROLS =====================
    private double turnSensitivity = 0.7;
    private static final double TURN_SENS_MIN  = 0.2;
    private static final double TURN_SENS_MAX  = 1.0;
    private static final double TURN_SENS_STEP = 0.1;
    private boolean g1DpadUpPrev   = false;
    private boolean g1DpadDnPrev   = false;

    private static final double DRIVE_SPEED_HIGH = 1.0;
    private static final double DRIVE_SPEED_LOW  = 0.4;
    private boolean slowMode  = false;
    private boolean g1YPrev   = false;

    // ===================== AIM + SNAP =====================
    private boolean aimEnabled = false; // toggled by G1.A
    private boolean g1APrev = false;

    // ODOM point-aim (fallback) gains (kept)
    private static final boolean AIM_INVERT_ROTATE = true;
    private static final double GOAL_AIM_HEADING_OFFSET_DEG = 90.0;
    private static final double GOAL_AIM_KP = 0.035;
    private static final double GOAL_AIM_KD = 0.0020;
    private static final double GOAL_AIM_MAX_POWER = 0.85;
    private static final double GOAL_AIM_DEADBAND_DEG = 1.0;

    // 180 latch
    private static final double AIM_180_LATCH_ENTER_DEG = 170.0;
    private static final double AIM_180_LATCH_EXIT_DEG  = 160.0;
    private boolean aim180Latched = false;
    private int aim180Sign = 1;

    private double lastAimErrDeg = 0.0;
    private double aimErrDegTelem = 0.0;
    private double aimDesiredDegTelem = 0.0;

    // Momentary snap (no hold)
    private boolean g1LTPrev = false;
    private boolean g1RTPrev = false;
    private boolean g1BPrev  = false;

    private boolean snapActive = false;
    private double snapTargetDeg = 0.0;
    private double snapEndTimeSec = 0.0;

    private static final double SNAP_TIMEOUT_SEC = 1.25;
    private static final double SNAP_KP = 0.040;
    private static final double SNAP_MAX_POWER = 0.90;
    private static final double SNAP_MIN_POWER = 0.20;
    private static final double SNAP_STOP_DEG  = 1.2;

    // Snap targets (adjustable)
    private static final double SNAP_B_TARGET_DEG  = 0.0;   // G1.B
    private static final double SNAP_RT_TARGET_DEG = -22;  // G1 right trigger
    private static final double SNAP_LT_TARGET_DEG = 22; // G1 left trigger

    @Override
    public void runOpMode() {

        // Drive
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Mechs
        intakeMotor  = hardwareMap.get(DcMotor.class,   "intake");
        feederMotor  = hardwareMap.get(DcMotor.class,   "feeder");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // LEDs
        ledLeft  = hardwareMap.get(Servo.class, "ledLeft");
        ledRight = hardwareMap.get(Servo.class, "ledRight");
        forceServoPwmRangeIfPossible(ledLeft);
        forceServoPwmRangeIfPossible(ledRight);

        // Stoppers
        leftStop  = hardwareMap.get(Servo.class, "leftStop");
        rightStop = hardwareMap.get(Servo.class, "rightStop");

        // Pinpoint (UNCHANGED)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(20);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        feederMotor.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter encoder for velocity measurement (custom PIDF uses power)
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Stoppers startup positions (CLOSED)
        setStoppersClosed(true);
        stoppersClosed = true;
        stoppersClosedPrev = true;

        // initial LED
        setLedColor(LedColor.ORANGE);

        telemetry.addLine("NewTestTeleop:");
        telemetry.addLine("G2.B toggles shooter main");
        telemetry.addLine("G2.BACK toggles IDLE (default ON @ 1200 RPM)");
        telemetry.addLine("G2.Y HOLD = backspin MAX power (override)");
        telemetry.addLine("NOTE: IDLE does NOT engage target RPM / READY / LEDs");
        telemetry.addLine("G2.A holds open + max feed/intake; G2.X holds open + pulses feed/intake");
        telemetry.update();

        waitForStart();

        double lastLoopTimeSec = getRuntime();

        while (opModeIsActive()) {

            double now = getRuntime();
            double dt = now - lastLoopTimeSec;
            if (dt <= 0) dt = 0.02;
            if (dt > 0.2) dt = 0.02;
            lastLoopTimeSec = now;

            // ----- Update pose -----
            pinpoint.update();
            robotPose = pinpoint.getPosition();

            double headingRad = robotPose.getHeading(AngleUnit.RADIANS);
            double headingDeg = robotPose.getHeading(AngleUnit.DEGREES);
            double xMm = robotPose.getX(DistanceUnit.MM);
            double yMm = robotPose.getY(DistanceUnit.MM);

            // ----- Update Limelight -----
            updateLimelightOriginalFiltered();

            // ----- Compute lock target point (odom fallback) -----
            lockTargetXmm = GOAL_X_MM + LOCK_TARGET_OFFSET_X_MM;
            lockTargetYmm = GOAL_Y_MM + LOCK_TARGET_OFFSET_Y_MM;

            double odomDistM = Math.hypot(xMm - lockTargetXmm, yMm - lockTargetYmm) / 1000.0;

            // "mix between limelight and odometry", default to limelight
            double fusedDistM = fuseDistanceMeters(limelightHasTarget, limelightDistanceMeters, odomDistM);

            // ===================== STOPPERS (HOLD) =====================
            // Open while G2.A held OR G2.X held, else closed.
            boolean aHeld = gamepad2.a;
            boolean xHeld = gamepad2.x;
            boolean openHeld = aHeld || xHeld;

            boolean newClosed = !openHeld;
            if (newClosed != stoppersClosedPrev) {
                setStoppersClosed(newClosed);
                stoppersClosedPrev = newClosed;
            }
            stoppersClosed = newClosed;

            // ===================== DRIVER (G1) =====================
            if (gamepad1.dpad_up && !g1DpadUpPrev) turnSensitivity += TURN_SENS_STEP;
            if (gamepad1.dpad_down && !g1DpadDnPrev) turnSensitivity -= TURN_SENS_STEP;
            g1DpadUpPrev = gamepad1.dpad_up;
            g1DpadDnPrev = gamepad1.dpad_down;
            turnSensitivity = Range.clip(turnSensitivity, TURN_SENS_MIN, TURN_SENS_MAX);

            if (gamepad1.y && !g1YPrev) slowMode = !slowMode;
            g1YPrev = gamepad1.y;
            double driveSpeedScale = slowMode ? DRIVE_SPEED_LOW : DRIVE_SPEED_HIGH;

            // Aim toggle (G1.A)
            if (gamepad1.a && !g1APrev) {
                aimEnabled = !aimEnabled;

                lastAimErrDeg = 0.0;
                aim180Latched = false;
                tagLockStableCount = 0;

                if (aimEnabled) lockedTagIdWhenAim = limelightTagId;
                else lockedTagIdWhenAim = -1;
            }
            g1APrev = gamepad1.a;

            // Momentary snaps:
            boolean ltSnap = gamepad1.left_trigger > 0.5;
            boolean rtSnap = gamepad1.right_trigger > 0.5;
            boolean bSnap  = gamepad1.b;

            if (ltSnap && !g1LTPrev) startSnap(now, SNAP_LT_TARGET_DEG);
            if (rtSnap && !g1RTPrev) startSnap(now, SNAP_RT_TARGET_DEG);
            if (bSnap  && !g1BPrev)  startSnap(now, SNAP_B_TARGET_DEG);

            g1LTPrev = ltSnap;
            g1RTPrev = rtSnap;
            g1BPrev  = bSnap;

            // Field-centric translation
            double fieldY = -gamepad1.left_stick_y * driveSpeedScale;
            double fieldX =  gamepad1.left_stick_x * driveSpeedScale;

            // Default rotate = manual
            double rotate = gamepad1.right_stick_x * turnSensitivity;

            // Rotate priority
            boolean llAimUsed = false;
            if (snapActive) {
                rotate = updateSnapRotate(now, headingDeg);
            } else if (aimEnabled) {
                double llRotate = computeLimelightAimRotate();
                llAimUsed = !Double.isNaN(llRotate);
                rotate = llAimUsed
                        ? llRotate
                        : computeAimToPointRotate(headingDeg, xMm, yMm, lockTargetXmm, lockTargetYmm, dt);
            }

            // Field-centric transform
            double rotX = fieldX * Math.cos(-headingRad) - fieldY * Math.sin(-headingRad);
            double rotY = fieldX * Math.sin(-headingRad) + fieldY * Math.cos(-headingRad);

            mecanumDrive(rotY, rotX, rotate);

            // ===================== OPERATOR (G2) =====================

            // Idle toggle (G2.BACK)
            boolean g2Back = gamepad2.back;
            if (g2Back && !g2BackPrev) {
                idleEnabled = !idleEnabled;
                resetShooterController();
                // keep shooterPowerCmdPrev as-is so idle ramp doesn't jerk
            }
            g2BackPrev = g2Back;

            // Backspin override (G2.Y HOLD) - max power reverse
            boolean backspinHeld = gamepad2.y;

            // Shooter toggle (G2.B)
            if (gamepad2.b && !g2BPrev) {
                shooterOn = !shooterOn;
                resetShooterController();
                // optional: clear ramp memory when switching modes to avoid surprises
                // shooterPowerCmdPrev = 0.0;
            }
            g2BPrev = gamepad2.b;

            // AutoRPM toggle (G2 left stick button)
            boolean g2LS = gamepad2.left_stick_button;
            if (g2LS && !g2LSPrev) autoRpmEnabled = !autoRpmEnabled;
            g2LSPrev = g2LS;

            // Manual RPM adjust (only when autoRPM off) - dpad up/down
            if (!autoRpmEnabled) {
                if (gamepad2.dpad_up && !g2DpadUpPrev) shooterRpmManual += SHOOTER_RPM_STEP;
                if (gamepad2.dpad_down && !g2DpadDnPrev) shooterRpmManual -= SHOOTER_RPM_STEP;
            }
            g2DpadUpPrev = gamepad2.dpad_up;
            g2DpadDnPrev = gamepad2.dpad_down;
            shooterRpmManual = Range.clip(shooterRpmManual, 0.0, MAX_RPM);

            // AutoRPM gain tweak - dpad left/right
            if (gamepad2.dpad_right && !g2DpadRightPrev) shooterRpmSqrtGain += SHOOTER_RPM_SQRT_GAIN_STEP;
            if (gamepad2.dpad_left  && !g2DpadLeftPrev)  shooterRpmSqrtGain -= SHOOTER_RPM_SQRT_GAIN_STEP;
            g2DpadRightPrev = gamepad2.dpad_right;
            g2DpadLeftPrev  = gamepad2.dpad_left;
            shooterRpmSqrtGain = Range.clip(shooterRpmSqrtGain, 0.0, 5000.0);

            // --------------------- Intake / Feeder (manual) ---------------------
            double intakePower = 0.0;
            if (gamepad2.left_trigger > 0.05) intakePower = gamepad2.left_trigger * INTAKE_FWD_POWER;
            if (gamepad2.left_bumper)         intakePower = -INTAKE_REV_POWER;

            double feederPower = 0.0;
            if (gamepad2.right_trigger > 0.05) feederPower = gamepad2.right_trigger * FEEDER_FWD_POWER;
            if (gamepad2.right_bumper)         feederPower = -FEEDER_REV_POWER;

            // --------------------- Overrides: A hold beats X pulse ---------------------
            boolean xPulse = xHeld && !aHeld;

            if (aHeld) {
                pulseIsOn = false;
                intakePower = A_HOLD_INTAKE_POWER;
                feederPower = A_HOLD_FEEDER_POWER;
            } else if (xPulse) {
                if (xPulse && !g2XPrev) {
                    pulseStartTimeSec = now;
                }
                double cycle = PULSE_ON_SEC + PULSE_OFF_SEC;
                double t = now - pulseStartTimeSec;
                double phase = (cycle > 1e-6) ? (t % cycle) : 0.0;
                pulseIsOn = phase < PULSE_ON_SEC;

                if (pulseIsOn) {
                    intakePower = PULSE_INTAKE_POWER;
                    feederPower = PULSE_FEEDER_POWER;
                } else {
                    intakePower = 0.0;
                    feederPower = 0.0;
                }
            } else {
                pulseIsOn = false;
            }
            g2XPrev = xPulse;

            intakeMotor.setPower(intakePower);
            feederMotor.setPower(feederPower);

            // ===================== SHOOTER TARGETS (IMPORTANT CHANGE) =====================
            // Active target RPM exists ONLY when shooterOn is true.
            // Idle is separate open-loop feedforward and does NOT count as "target RPM" (so LEDs won't change).
            double shooterRpmTargetActive = 0.0;
            boolean farAutoMode = false;

            if (!backspinHeld && shooterOn) {
                if (autoRpmEnabled) {
                    double d = Math.max(0.0, fusedDistM);

                    if (d > FAR_DISTANCE_THRESHOLD_M) {
                        shooterRpmTargetActive = FAR_AUTO_RPM;
                        farAutoMode = true;
                    } else {
                        double autoRpm = AUTO_RPM_REF_RPM
                                + shooterRpmSqrtGain * (Math.sqrt(d) - Math.sqrt(AUTO_RPM_REF_DIST_M));
                        shooterRpmTargetActive = Range.clip(autoRpm, MIN_AUTO_RPM, MAX_RPM);
                        farAutoMode = false;
                    }
                } else {
                    shooterRpmTargetActive = shooterRpmManual;
                }
            }
            shooterRpmTargetActive = Range.clip(shooterRpmTargetActive, 0.0, MAX_RPM);

            boolean idleRequested = (!backspinHeld && !shooterOn && idleEnabled);

            // ===================== SHOOTER CONTROL =====================
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double shooterPowerCmd = 0.0;

            if (backspinHeld) {
                // MAX reverse power (hold)
                resetShooterController();
                shooterPowerCmdPrev = 0.0; // reset ramp memory so we don't "ramp from -1"
                shooterPowerCmd = -1.0;    // flip sign if motor direction is opposite
                shooterMotor.setPower(shooterPowerCmd);

            } else if (shooterOn) {
                // CLOSED-LOOP ONLY when shooter is active (target RPM engaged)
                double measuredTPS = shooterMotor.getVelocity();
                double targetTPS = (shooterRpmTargetActive > 1.0) ? rpmToTicksPerSecond(shooterRpmTargetActive) : 0.0;

                if (targetTPS > 0.0) {
                    double error = targetTPS - measuredTPS;

                    double fTerm = kF * targetTPS;
                    double pTerm = kP * error;
                    double dTerm = kD * ((error - shooterLastError) / dt);
                    double prelim = fTerm + pTerm + dTerm;

                    boolean wouldSaturateHigh = prelim >= 1.0;
                    boolean wouldSaturateLow  = prelim <= 0.0;

                    boolean allowIntegrate = (!wouldSaturateHigh && !wouldSaturateLow)
                            || (wouldSaturateHigh && error < 0)
                            || (wouldSaturateLow  && error > 0);

                    if (allowIntegrate) {
                        shooterIntegral += error * dt;
                        shooterIntegral = Range.clip(shooterIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
                    }

                    double iTerm = kI * shooterIntegral;
                    shooterPowerCmd = Range.clip(prelim + iTerm, 0.0, 1.0);

                    shooterMotor.setPower(shooterPowerCmd);
                    shooterLastError = error;

                    shooterPowerCmdPrev = shooterPowerCmd; // keep continuity for later ramp-to-idle
                } else {
                    resetShooterController();
                    shooterPowerCmd = rampTowards(shooterPowerCmdPrev, 0.0, IDLE_POWER_RAMP_PER_SEC * dt);
                    shooterMotor.setPower(shooterPowerCmd);
                    shooterPowerCmdPrev = shooterPowerCmd;
                }

            } else if (idleRequested) {
                // IDLE: open-loop feedforward ONLY (no PID / no target RPM / no READY / no LED impact)
                resetShooterController();

                double idleTPS = rpmToTicksPerSecond(IDLE_RPM);
                double idleFF = (kF * idleTPS) + IDLE_POWER_FUDGE; // basic feedforward
                idleFF = Range.clip(idleFF, 0.0, 1.0);

                shooterPowerCmd = rampTowards(shooterPowerCmdPrev, idleFF, IDLE_POWER_RAMP_PER_SEC * dt);
                shooterMotor.setPower(shooterPowerCmd);
                shooterPowerCmdPrev = shooterPowerCmd;

            } else {
                // Fully off: ramp down a bit to be brownout-friendly
                resetShooterController();
                shooterPowerCmd = rampTowards(shooterPowerCmdPrev, 0.0, IDLE_POWER_RAMP_PER_SEC * dt);
                shooterMotor.setPower(shooterPowerCmd);
                shooterPowerCmdPrev = shooterPowerCmd;
            }

            // Shooter measured + ready
            double shooterRpmMeas = getShooterRpmMeasured();

            // LEDs should only consider "active shooter" (shooterOn or backspin), NOT idle.
            boolean shooterActiveForLeds = backspinHeld || shooterOn;

            boolean shooterReady = shooterOn && !backspinHeld
                    && (shooterRpmTargetActive > 1.0)
                    && Math.abs(shooterRpmMeas - shooterRpmTargetActive) <= 150.0;

            // LEDs
            updateLeds(stoppersClosed, shooterActiveForLeds, shooterReady);

            // ===================== TELEMETRY =====================
            telemetry.addData("Pose x/y (mm)", "%.0f / %.0f", xMm, yMm);
            telemetry.addData("Heading (deg)", "%.1f", headingDeg);

            telemetry.addData("Stoppers", openHeld ? "OPEN (A/X)" : "CLOSED");
            telemetry.addData("A max feed/intake", aHeld);
            telemetry.addData("X pulse", xPulse ? (pulseIsOn ? "ON" : "OFF") : "OFF");

            telemetry.addData("OdomDist (m)", "%.2f", odomDistM);
            telemetry.addData("LL Dist (m)", "%.2f", limelightDistanceMeters);
            telemetry.addData("FusedDist (m)", "%.2f", fusedDistM);

            telemetry.addLine("\nLimelight");
            telemetry.addData("HasTarget", limelightHasTarget);
            telemetry.addData("TagId (cur/locked)", "%d / %d", limelightTagId, lockedTagIdWhenAim);
            telemetry.addData("tx raw/filt", "%.2f / %.2f", limelightTxRawDeg, limelightTxDeg);

            telemetry.addLine("\nAim/Snap");
            telemetry.addData("AimEnabled (G1.A)", aimEnabled);
            telemetry.addData("AimSource", (!aimEnabled) ? "OFF" : (llAimUsed ? "LIMELIGHT" : "ODOM-FALLBACK"));
            telemetry.addData("stableCount", "%d/%d", tagLockStableCount, TAG_LOCK_STABLE_CYCLES);
            telemetry.addData("SnapActive", snapActive);

            telemetry.addLine("\nShooter");
            telemetry.addData("IdleEnabled (G2.BACK)", idleEnabled);
            telemetry.addData("IdleRequested", idleRequested);
            telemetry.addData("BackspinHeld (G2.Y)", backspinHeld);
            telemetry.addData("AutoRPM", autoRpmEnabled);
            telemetry.addData("FAR auto mode", farAutoMode);
            telemetry.addData("ShooterMainOn (G2.B)", shooterOn);
            telemetry.addData("RPM tgtActive/meas", "%.0f / %.0f", shooterRpmTargetActive, shooterRpmMeas);
            telemetry.addData("IdleRPM (const)", "%.0f", IDLE_RPM);
            telemetry.addData("AutoGain (RPM/√m)", "%.1f", shooterRpmSqrtGain);
            telemetry.addData("Ready", shooterReady);
            telemetry.addData("ShooterPwr", "%.3f", shooterPowerCmd);

            telemetry.addLine("\nLED");
            telemetry.addData("LED", ledState.name());

            telemetry.update();
        }

        if (limelight != null) limelight.stop();
    }

    // ===================== DISTANCE FUSION =====================
    private double fuseDistanceMeters(boolean llHasTarget, double llDistM, double odomDistM) {
        if (llHasTarget && llDistM > 0.05) {
            return DIST_FUSION_LL_WEIGHT * llDistM + (1.0 - DIST_FUSION_LL_WEIGHT) * odomDistM;
        }
        return odomDistM;
    }

    // ===================== LIMELIGHT AIM (Teleop2 algorithm + far bias) =====================
    private double computeLimelightAimRotate() {
        if (!limelightHasTarget) {
            tagLockStableCount = 0;
            return Double.NaN;
        }

        if (LIMELIGHT_LOCK_TO_TAG_ID && lockedTagIdWhenAim != -1 && limelightTagId != -1) {
            if (limelightTagId != lockedTagIdWhenAim) {
                tagLockStableCount = 0;
                return Double.NaN;
            }
        }

        double aimOffsetDeg = 0.0;
        if (tagBiasEnabled && limelightDistanceMeters > FAR_DISTANCE_THRESHOLD_M && limelightTagId != -1) {
            if (limelightTagId == 20) aimOffsetDeg = TAG_AIM_BIAS_DEG;
            else if (limelightTagId == 24) aimOffsetDeg = -TAG_AIM_BIAS_DEG;
        }

        double bearingDeg = limelightTxDeg - aimOffsetDeg;

        if (Math.abs(bearingDeg) <= TAG_AIM_DEADBAND_DEG) tagLockStableCount++;
        else tagLockStableCount = 0;

        if (tagLockStableCount >= TAG_LOCK_STABLE_CYCLES) {
            return 0.0;
        } else {
            double maxTurn = (Math.abs(bearingDeg) <= (TAG_AIM_DEADBAND_DEG * 2.0))
                    ? TAG_MAX_TURN_POWER_FINE : TAG_MAX_TURN_POWER;

            return Range.clip(TAG_TURN_KP * bearingDeg, -maxTurn, maxTurn);
        }
    }

    // ===================== LIMELIGHT FILTER (Teleop2) =====================
    private void updateLimelightOriginalFiltered() {
        limelightHasTarget = false;
        limelightTagId = -1;

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
        double heightDiff = TARGET_HEIGHT_M - LIMELIGHT_HEIGHT_M;

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

    // ===================== LEDs =====================
    private void forceServoPwmRangeIfPossible(Servo s) {
        if (s instanceof ServoImplEx) {
            ServoImplEx ex = (ServoImplEx) s;
            ex.setPwmRange(new PwmControl.PwmRange((int) LED_PWM_MIN_US, (int) LED_PWM_MAX_US));
            ex.setPwmEnable();
        }
    }

    private double pulseUsToServoPos(double pulseUs) {
        double p = Range.clip(pulseUs, LED_PWM_MIN_US, LED_PWM_MAX_US);
        return (p - LED_PWM_MIN_US) / (LED_PWM_MAX_US - LED_PWM_MIN_US);
    }

    private double basePulseForColor(LedColor c) {
        switch (c) {
            case RED:    return LED_RED_US_BASE;
            case ORANGE: return LED_ORANGE_US_BASE;
            case YELLOW: return LED_YELLOW_US_BASE;
            case GREEN:  return LED_GREEN_US_BASE;
        }
        return LED_ORANGE_US_BASE;
    }

    private void setLedColor(LedColor c) {
        ledState = c;
        double pulseUs = basePulseForColor(c) + ledPulseOffsetUs;
        double pos = pulseUsToServoPos(pulseUs);
        ledLeft.setPosition(pos);
        ledRight.setPosition(pos);
    }

    private void updateLeds(boolean stoppersClosed, boolean shooterOnLike, boolean shooterReady) {
        if (!shooterOnLike) {
            setLedColor(stoppersClosed ? LedColor.RED : LedColor.ORANGE);
            return;
        }
        if (shooterReady) {
            setLedColor(stoppersClosed ? LedColor.YELLOW : LedColor.GREEN);
            return;
        }
        setLedColor(stoppersClosed ? LedColor.RED : LedColor.ORANGE);
    }

    // ===================== STOPPERS =====================
    private void setStoppersClosed(boolean closed) {
        if (closed) {
            leftStop.setPosition(LEFT_STOP_ZERO_POS);
            rightStop.setPosition(RIGHT_STOP_LEFT_POS);
        } else {
            leftStop.setPosition(LEFT_STOP_LEFT_POS);
            rightStop.setPosition(RIGHT_STOP_RIGHT_POS);
        }
    }

    // ===================== SNAP =====================
    private void startSnap(double nowSec, double targetDeg) {
        snapActive = true;
        snapTargetDeg = wrapAngleDeg(targetDeg);
        snapEndTimeSec = nowSec + SNAP_TIMEOUT_SEC;
    }

    private double updateSnapRotate(double nowSec, double headingDeg) {
        if (!snapActive) return 0.0;

        if (nowSec >= snapEndTimeSec) {
            snapActive = false;
            return 0.0;
        }

        double errDeg = wrapAngleDeg(headingDeg - snapTargetDeg);

        if (Math.abs(errDeg) <= SNAP_STOP_DEG) {
            snapActive = false;
            return 0.0;
        }

        double cmd = SNAP_KP * errDeg;
        cmd = Range.clip(cmd, -SNAP_MAX_POWER, SNAP_MAX_POWER);

        if (Math.abs(cmd) < SNAP_MIN_POWER) cmd = Math.signum(cmd) * SNAP_MIN_POWER;
        return cmd;
    }

    // ===================== AIM (ODOM fallback to arbitrary point) =====================
    private double computeAimToPointRotate(double headingDeg, double xMm, double yMm,
                                           double targetXmm, double targetYmm, double dt) {
        double dx = -targetXmm + xMm;
        double dy =  targetYmm - yMm;

        if (Math.hypot(dx, dy) < 5.0) {
            lastAimErrDeg = 0.0;
            aim180Latched = false;
            aimErrDegTelem = 0.0;
            aimDesiredDegTelem = headingDeg;
            return 0.0;
        }

        double desiredDeg = Math.toDegrees(Math.atan2(dx, dy));
        desiredDeg = wrapAngleDeg(desiredDeg + GOAL_AIM_HEADING_OFFSET_DEG);

        double errDeg = wrapAngleDeg(desiredDeg - headingDeg);

        double absErr = Math.abs(errDeg);
        if (!aim180Latched && absErr >= AIM_180_LATCH_ENTER_DEG) {
            aim180Latched = true;
            aim180Sign = (errDeg >= 0.0) ? 1 : -1;
            if (aim180Sign == 0) aim180Sign = 1;
        } else if (aim180Latched && absErr <= AIM_180_LATCH_EXIT_DEG) {
            aim180Latched = false;
        }

        if (aim180Latched) errDeg = 180.0 * aim180Sign;

        double derr = (errDeg - lastAimErrDeg) / Math.max(1e-3, dt);
        lastAimErrDeg = errDeg;

        double cmd;
        if (Math.abs(errDeg) <= GOAL_AIM_DEADBAND_DEG) cmd = 0.0;
        else {
            cmd = GOAL_AIM_KP * errDeg + GOAL_AIM_KD * derr;
            cmd = Range.clip(cmd, -GOAL_AIM_MAX_POWER, GOAL_AIM_MAX_POWER);
        }

        if (AIM_INVERT_ROTATE) cmd = -cmd;

        aimDesiredDegTelem = desiredDeg;
        aimErrDegTelem = errDeg;

        return cmd;
    }

    // ===================== SHOOTER HELPERS =====================
    private void resetShooterController() {
        shooterIntegral = 0.0;
        shooterLastError = 0.0;
    }

    private static double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * SHOOTER_TICKS_PER_REV;
    }

    private double getShooterRpmMeasured() {
        double ticksPerSec = shooterMotor.getVelocity();
        return ticksPerSec * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    private static double rampTowards(double current, double target, double maxDelta) {
        if (maxDelta < 0) maxDelta = -maxDelta;
        double delta = target - current;
        if (Math.abs(delta) <= maxDelta) return target;
        return current + Math.signum(delta) * maxDelta;
    }

    // ===================== MECANUM =====================
    private void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);

        double leftFrontPower  = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower   = (forward - strafe + rotate) / denominator;
        double rightBackPower  = (forward + strafe - rotate) / denominator;

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }

    // robust wrap to (-180, 180]
    private static double wrapAngleDeg(double angle) {
        angle = ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        if (angle == -180.0) angle = 180.0;
        return angle;
    }
}