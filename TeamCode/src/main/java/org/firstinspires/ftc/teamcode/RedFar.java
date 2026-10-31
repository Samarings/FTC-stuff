package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RedFar", group = "A")
public class RedFar extends OpMode {

    // ===================== PEDRO =====================
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        INIT_POSE,

        START_TO_SHOOT_FOLLOW,
        SPINUP_PRELOAD,
        SHOOTING_PRELOAD,
        BRAKING_PRELOAD,

        // 1ST SET (collect) -> shoot
        SHOOT_TO_1STSET_FOLLOW,
        FIRSTSET_TO_INTAKED_FIRSTSET_FOLLOW,
        INTAKED_FIRSTSET_TO_SHOOT_FOLLOW,
        SPINUP_1,
        SHOOTING_1,
        BRAKING_1,

        // 2ND SET (collect) -> shoot
        SHOOT_TO_2NDSET_FOLLOW,
        SECONDSET_TO_FIRSTBARHIT_FOLLOW,
        FIRSTBARHIT_TO_SHOOT_FOLLOW,
        SPINUP_2,
        SHOOTING_2,
        BRAKING_2,

        // BAR CYCLES LAST
        SHOOT_TO_BAR_FOLLOW,
        AT_BAR_INTAKE_WAIT,
        BAR_TO_SHOOT_FOLLOW,
        SPINUP_BAR,
        SHOOTING_BAR,
        BRAKING_BAR,

        // END
        SHOOT_TO_END_FOLLOW,
        DONE
    }
    private PathState pathState;

    // ===================== POSES (INCHES) =====================
    private final Pose startPose = new Pose(-23.228, -117.004, Math.toRadians(0));
    private final Pose shootPose = new Pose(-30.675, -109.866, Math.toRadians(67.57));

    private final Pose secondSetPose   = new Pose(-13.051, -90.471, Math.toRadians(0)); // <-- CHANGE ME
    private final Pose firstBarHitPose = new Pose(11.981,  -90.471, Math.toRadians(0));    // <-- CHANGE ME

    private final Pose barPose = new Pose(11.847, -88.681, Math.toRadians(22.18));   // <-- CHANGE ME

    private final Pose firstSetPose        = new Pose(-2, -114.343, Math.toRadians(0));  // <-- CHANGE ME
    private final Pose intakedFirstSetPose = new Pose(17, -114.343, Math.toRadians(0)); // <-- CHANGE ME

    private final Pose endPose = new Pose(-14, -97, Math.toRadians(67)); // <-- CHANGE ME

    // ===================== PATHS =====================
    private PathChain startToShoot;

    // 1st set
    private PathChain shootTo1stSet;
    private PathChain firstSetToIntaked1stSet;
    private PathChain intaked1stSetToShoot;

    // 2nd set
    private PathChain shootToSecondSet;
    private PathChain secondSetToFirstBarHit;
    private PathChain firstBarHitToShoot;

    // bar (STRAIGHT)
    private PathChain shootToBar;
    private PathChain barToShoot;

    // end
    private PathChain shootToEnd;

    // ===================== HARDWARE =====================
    private DcMotor intakeMotor;
    private DcMotor feederMotor;
    private DcMotorEx shooterMotor;

    // ===================== PINPOINT =====================
    private GoBildaPinpointDriver pinpoint;

    // Non-blocking pinpoint init
    private boolean poseInitStarted = false;
    private double poseInitStartSec = 0.0;
    private static final double PINPOINT_RESET_WAIT_SEC = 0.30;

    // ===================== STOPPERS =====================
    private Servo leftStop, rightStop;
    private boolean stoppersClosed = false;

    private static final double LEFT_STOP_ZERO_POS   = 0.50; // closed
    private static final double LEFT_STOP_LEFT_POS   = 0.00; // open
    private static final double RIGHT_STOP_LEFT_POS  = 0.00; // closed
    private static final double RIGHT_STOP_RIGHT_POS = 1.00; // open

    // ===================== SHOOTER CONTROL (CUSTOM PIDF-IN-POWER) =====================
    private double kP = 0.015;
    private double kI = 0.0;
    private double kD = 0.0001;
    private double kF = 0.000422143;

    private static final double INTEGRAL_CLAMP = 5000.0;
    private double shooterIntegral = 0.0;
    private double shooterLastError = 0.0;
    private double shooterPowerCmd = 0.0;

    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // ===================== SETTINGS =====================
    // FAR shooting RPM (CHANGE ME)
    private static final double FAR_SHOOT_TARGET_RPM = 3000.0;  // <-- CHANGE ME
    private static final double SHOOT_READY_TOL_RPM = 120;
    private static final int    SHOOT_READY_STABLE_CYCLES = 5;

    private static final double PRESPIN_DISTANCE_IN = 30;
    private static final double SPINUP_TIMEOUT_SEC = 5;

    private static final double SHOOT_DURATION_SEC = 1.15;

    // pulsed feed instead of holding
    private static final double FEEDER_PULSE_POWER = 1.0;
    private static final double INTAKE_PULSE_POWER = 1.0;
    private static final double PULSE_ON_SEC  = 0.11;  // <-- tune
    private static final double PULSE_OFF_SEC = 0.08;  // <-- tune

    // collect
    private static final double FEEDER_COLLECT_POWER = 1.0;
    private static final double INTAKE_COLLECT_POWER = 1.0;

    // bar wait intake
    private static final double BAR_INTAKE_TIME_SEC = 1.2;
    private static final int    BAR_CYCLES = 2;

    // kick accel
    private static final double KICK_TIME_SEC = 0.15;
    private static final double KICK_POWER    = 1.0;

    private boolean shooterKickActive = false;
    private double  shooterKickEndSec = 0.0;
    private double  lastShooterCmdRpm = 0.0;

    // harsh brake
    private boolean shooterBrakeActive = false;
    private double  shooterBrakeEndTime = 0.0;
    private int     shooterBrakeInitialSign = 0;

    private static final double SHOOTER_BRAKE_MIN_POWER = 0.8;
    private static final double SHOOTER_BRAKE_MAX_POWER = 1.00;
    private static final double SHOOTER_BRAKE_MIN_TIME  = 0.16;
    private static final double SHOOTER_BRAKE_MAX_TIME  = 1.50;
    private static final double SHOOTER_BRAKE_RPM_FULL  = 4500.0;
    private static final double SHOOTER_BRAKE_STOP_RPM  = 100.0;

    // ===================== INTERNAL STATE =====================
    private boolean pathStarted = false;

    private boolean preSpinStarted = false;
    private double  preSpinStartSec = -1.0;
    private int     shooterReadyCount = 0;

    private double shootingStartSec = -1.0;
    private double lastLoopSec = -1.0;

    // pulse state
    private boolean pulseOn = false;
    private double  nextPulseToggleSec = 0.0;

    // bar bookkeeping
    private int barCycleCount = 0;
    private double barWaitStartSec = -1.0;

    // ===================== PATH BUILD =====================
    private void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // 1st set
        shootTo1stSet = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstSetPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstSetPose.getHeading())
                .build();

        firstSetToIntaked1stSet = follower.pathBuilder()
                .addPath(new BezierLine(firstSetPose, intakedFirstSetPose))
                .setLinearHeadingInterpolation(firstSetPose.getHeading(), intakedFirstSetPose.getHeading())
                .build();

        intaked1stSetToShoot = follower.pathBuilder()
                .addPath(new BezierLine(intakedFirstSetPose, shootPose))
                .setLinearHeadingInterpolation(intakedFirstSetPose.getHeading(), shootPose.getHeading())
                .build();

        // 2nd set
        shootToSecondSet = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondSetPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondSetPose.getHeading())
                .build();

        secondSetToFirstBarHit = follower.pathBuilder()
                .addPath(new BezierLine(secondSetPose, firstBarHitPose))
                .setLinearHeadingInterpolation(secondSetPose.getHeading(), firstBarHitPose.getHeading())
                .build();

        firstBarHitToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstBarHitPose, shootPose))
                .setLinearHeadingInterpolation(firstBarHitPose.getHeading(), shootPose.getHeading())
                .build();

        // bar (straight)
        shootToBar = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, barPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), barPose.getHeading())
                .build();

        barToShoot = follower.pathBuilder()
                .addPath(new BezierLine(barPose, shootPose))
                .setLinearHeadingInterpolation(barPose.getHeading(), shootPose.getHeading())
                .build();

        // end
        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    // ===================== PINPOINT HARD-SET (NON-BLOCKING) =====================
    private void beginHardSetPinpointAndPedroToStart(double nowSec) {
        pinpoint.resetPosAndIMU();
        poseInitStarted = true;
        poseInitStartSec = nowSec;
    }

    private boolean finishHardSetPinpointAndPedroToStart(double nowSec) {
        if (!poseInitStarted) return false;
        if (nowSec - poseInitStartSec < PINPOINT_RESET_WAIT_SEC) return false;

        Pose2D p2d = new Pose2D(
                DistanceUnit.INCH,
                startPose.getX(),
                startPose.getY(),
                AngleUnit.RADIANS,
                startPose.getHeading()
        );
        pinpoint.setPosition(p2d);

        // sync Pedro estimate too
        follower.setPose(startPose);

        return true;
    }

    private void enterSpinupState(double nowSec) {
        preSpinStarted = false;
        preSpinStartSec = -1.0;
        shooterReadyCount = 0;

        shooterBrakeActive = false;
        shooterKickActive = false;

        stopPulse();
        resetShooterController();
    }

    private void enterShootingState(double nowSec) {
        shootingStartSec = nowSec;
        startPulse(nowSec);
    }

    // ===================== INIT =====================
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

        // Pinpoint setup (match your teleop)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        intakeMotor  = hardwareMap.get(DcMotor.class, "intake");
        feederMotor  = hardwareMap.get(DcMotor.class, "feeder");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        leftStop  = hardwareMap.get(Servo.class, "leftStop");
        rightStop = hardwareMap.get(Servo.class, "rightStop");
        setStoppersClosed(true);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        feederMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setPower(0.0);
        feederMotor.setPower(0.0);
        commandShooterRpm(0.0, 0.0, 0.02);

        poseInitStarted = false;
        pathStarted = false;

        barCycleCount = 0;
        barWaitStartSec = -1.0;

        pathState = PathState.INIT_POSE;

        telemetry.addLine("AutoFarPulse_1st2ndBar_Pinpoint");
        telemetry.addLine("INIT_POSE: resetPosAndIMU() + setPosition(startPose) + follower.setPose(startPose) (non-blocking).");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        lastLoopSec = getRuntime();

        poseInitStarted = false;
        pathStarted = false;
        enterSpinupState(getRuntime());
    }

    // ===================== LOOP =====================
    @Override
    public void loop() {
        follower.update();
        double nowSec = getRuntime();

        double dt = (lastLoopSec < 0) ? 0.02 : (nowSec - lastLoopSec);
        if (dt <= 0) dt = 0.02;
        if (dt > 0.20) dt = 0.02;
        lastLoopSec = nowSec;

        double intakeCmd = 0.0;
        double feederCmd = 0.0;

        switch (pathState) {

            case INIT_POSE: {
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);

                // keep mechs off
                intakeCmd = 0.0;
                feederCmd = 0.0;

                if (!poseInitStarted) {
                    beginHardSetPinpointAndPedroToStart(nowSec);
                }

                if (finishHardSetPinpointAndPedroToStart(nowSec)) {
                    pathStarted = false;
                    enterSpinupState(nowSec);
                    pathState = PathState.START_TO_SHOOT_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;
            }

            // ---------- INITIAL: go to shoot pose ----------
            case START_TO_SHOOT_FOLLOW:
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    follower.followPath(startToShoot, true);
                    pathStarted = true;

                    enterSpinupState(nowSec);
                    pathState = PathState.SPINUP_PRELOAD;
                    pathTimer.resetTimer();
                }
                break;

            case SPINUP_PRELOAD: {
                setStoppersClosed(true);

                double dist = distToPoseIn(follower.getPose(), shootPose);
                if (!preSpinStarted && dist <= PRESPIN_DISTANCE_IN) {
                    preSpinStarted = true;
                    preSpinStartSec = nowSec;
                }

                if (preSpinStarted) {
                    commandShooterRpm(FAR_SHOOT_TARGET_RPM, nowSec, dt);

                    double rpmMeas = getShooterRpmMeasured();
                    shooterReadyCount = (Math.abs(rpmMeas - FAR_SHOOT_TARGET_RPM) <= SHOOT_READY_TOL_RPM)
                            ? (shooterReadyCount + 1) : 0;
                } else {
                    commandShooterRpm(0.0, nowSec, dt);
                    shooterReadyCount = 0;
                }

                if (!follower.isBusy()) {
                    boolean ready = shooterReadyCount >= SHOOT_READY_STABLE_CYCLES;
                    double spinupElapsed = (preSpinStartSec > 0.0) ? (nowSec - preSpinStartSec) : 0.0;
                    boolean timedOut = preSpinStarted && (spinupElapsed >= SPINUP_TIMEOUT_SEC);

                    if (ready || timedOut) {
                        enterShootingState(nowSec);
                        pathState = PathState.SHOOTING_PRELOAD;
                        pathTimer.resetTimer();
                    }
                }
                break;
            }

            case SHOOTING_PRELOAD:
                setStoppersClosed(false);
                commandShooterRpm(FAR_SHOOT_TARGET_RPM, nowSec, dt);

                updatePulse(nowSec);
                if (pulseOn) {
                    feederCmd = FEEDER_PULSE_POWER;
                    intakeCmd = INTAKE_PULSE_POWER;
                }

                if (nowSec - shootingStartSec >= SHOOT_DURATION_SEC) {
                    stopPulse();
                    feederCmd = 0.0;
                    intakeCmd = 0.0;

                    startShooterBrake(nowSec);
                    pathState = PathState.BRAKING_PRELOAD;
                    pathTimer.resetTimer();
                }
                break;

            case BRAKING_PRELOAD:
                setStoppersClosed(true);
                updateShooterBrake(nowSec);

                if (!shooterBrakeActive) {
                    pathStarted = false;
                    pathState = PathState.SHOOT_TO_1STSET_FOLLOW; // 1st set first
                    pathTimer.resetTimer();
                }
                break;

            // ---------- 1ST SET ----------
            case SHOOT_TO_1STSET_FOLLOW:
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    follower.followPath(shootTo1stSet, true);
                    pathStarted = true;
                    pathTimer.resetTimer();
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.FIRSTSET_TO_INTAKED_FIRSTSET_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;

            case FIRSTSET_TO_INTAKED_FIRSTSET_FOLLOW:
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    setStoppersClosed(true);
                    follower.followPath(firstSetToIntaked1stSet, true);
                    pathStarted = true;
                    pathTimer.resetTimer();
                }

                if (follower.isBusy()) {
                    setStoppersClosed(true);
                    feederCmd = FEEDER_COLLECT_POWER;
                    intakeCmd = INTAKE_COLLECT_POWER;
                } else {
                    feederCmd = 0.0;
                    intakeCmd = 0.0;
                    setStoppersClosed(false);

                    pathStarted = false;
                    enterSpinupState(nowSec);
                    pathState = PathState.INTAKED_FIRSTSET_TO_SHOOT_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;

            case INTAKED_FIRSTSET_TO_SHOOT_FOLLOW:
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    follower.followPath(intaked1stSetToShoot, true);
                    pathStarted = true;
                    pathState = PathState.SPINUP_1;
                    pathTimer.resetTimer();
                }
                break;

            case SPINUP_1: {
                setStoppersClosed(true);

                double dist = distToPoseIn(follower.getPose(), shootPose);
                if (!preSpinStarted && dist <= PRESPIN_DISTANCE_IN) {
                    preSpinStarted = true;
                    preSpinStartSec = nowSec;
                }

                if (preSpinStarted) {
                    commandShooterRpm(FAR_SHOOT_TARGET_RPM, nowSec, dt);
                    double rpmMeas = getShooterRpmMeasured();
                    shooterReadyCount = (Math.abs(rpmMeas - FAR_SHOOT_TARGET_RPM) <= SHOOT_READY_TOL_RPM)
                            ? (shooterReadyCount + 1) : 0;
                } else {
                    commandShooterRpm(0.0, nowSec, dt);
                    shooterReadyCount = 0;
                }

                if (!follower.isBusy()) {
                    boolean ready = shooterReadyCount >= SHOOT_READY_STABLE_CYCLES;
                    double spinupElapsed = (preSpinStartSec > 0.0) ? (nowSec - preSpinStartSec) : 0.0;
                    boolean timedOut = preSpinStarted && (spinupElapsed >= SPINUP_TIMEOUT_SEC);

                    if (ready || timedOut) {
                        enterShootingState(nowSec);
                        pathState = PathState.SHOOTING_1;
                        pathTimer.resetTimer();
                    }
                }
                break;
            }

            case SHOOTING_1:
                setStoppersClosed(false);
                commandShooterRpm(FAR_SHOOT_TARGET_RPM, nowSec, dt);

                updatePulse(nowSec);
                if (pulseOn) {
                    feederCmd = FEEDER_PULSE_POWER;
                    intakeCmd = INTAKE_PULSE_POWER;
                }

                if (nowSec - shootingStartSec >= SHOOT_DURATION_SEC) {
                    stopPulse();
                    feederCmd = 0.0;
                    intakeCmd = 0.0;

                    startShooterBrake(nowSec);
                    pathState = PathState.BRAKING_1;
                    pathTimer.resetTimer();
                }
                break;

            case BRAKING_1:
                setStoppersClosed(true);
                updateShooterBrake(nowSec);

                if (!shooterBrakeActive) {
                    pathStarted = false;
                    pathState = PathState.SHOOT_TO_2NDSET_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;

            // ---------- 2ND SET ----------
            case SHOOT_TO_2NDSET_FOLLOW:
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    follower.followPath(shootToSecondSet, true);
                    pathStarted = true;
                    pathTimer.resetTimer();
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.SECONDSET_TO_FIRSTBARHIT_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;

            case SECONDSET_TO_FIRSTBARHIT_FOLLOW:
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    setStoppersClosed(true);
                    follower.followPath(secondSetToFirstBarHit, true);
                    pathStarted = true;
                    pathTimer.resetTimer();
                }

                if (follower.isBusy()) {
                    setStoppersClosed(true);
                    feederCmd = FEEDER_COLLECT_POWER;
                    intakeCmd = INTAKE_COLLECT_POWER;
                } else {
                    feederCmd = 0.0;
                    intakeCmd = 0.0;
                    setStoppersClosed(false);

                    pathStarted = false;
                    enterSpinupState(nowSec);
                    pathState = PathState.FIRSTBARHIT_TO_SHOOT_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;

            case FIRSTBARHIT_TO_SHOOT_FOLLOW:
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    follower.followPath(firstBarHitToShoot, true);
                    pathStarted = true;
                    pathState = PathState.SPINUP_2;
                    pathTimer.resetTimer();
                }
                break;

            case SPINUP_2: {
                setStoppersClosed(true);

                double dist = distToPoseIn(follower.getPose(), shootPose);
                if (!preSpinStarted && dist <= PRESPIN_DISTANCE_IN) {
                    preSpinStarted = true;
                    preSpinStartSec = nowSec;
                }

                if (preSpinStarted) {
                    commandShooterRpm(FAR_SHOOT_TARGET_RPM, nowSec, dt);
                    double rpmMeas = getShooterRpmMeasured();
                    shooterReadyCount = (Math.abs(rpmMeas - FAR_SHOOT_TARGET_RPM) <= SHOOT_READY_TOL_RPM)
                            ? (shooterReadyCount + 1) : 0;
                } else {
                    commandShooterRpm(0.0, nowSec, dt);
                    shooterReadyCount = 0;
                }

                if (!follower.isBusy()) {
                    boolean ready = shooterReadyCount >= SHOOT_READY_STABLE_CYCLES;
                    double spinupElapsed = (preSpinStartSec > 0.0) ? (nowSec - preSpinStartSec) : 0.0;
                    boolean timedOut = preSpinStarted && (spinupElapsed >= SPINUP_TIMEOUT_SEC);

                    if (ready || timedOut) {
                        enterShootingState(nowSec);
                        pathState = PathState.SHOOTING_2;
                        pathTimer.resetTimer();
                    }
                }
                break;
            }

            case SHOOTING_2:
                setStoppersClosed(false);
                commandShooterRpm(FAR_SHOOT_TARGET_RPM, nowSec, dt);

                updatePulse(nowSec);
                if (pulseOn) {
                    feederCmd = FEEDER_PULSE_POWER;
                    intakeCmd = INTAKE_PULSE_POWER;
                }

                if (nowSec - shootingStartSec >= SHOOT_DURATION_SEC) {
                    stopPulse();
                    feederCmd = 0.0;
                    intakeCmd = 0.0;

                    startShooterBrake(nowSec);
                    pathState = PathState.BRAKING_2;
                    pathTimer.resetTimer();
                }
                break;

            case BRAKING_2:
                setStoppersClosed(true);
                updateShooterBrake(nowSec);

                if (!shooterBrakeActive) {
                    barCycleCount = 0;
                    pathStarted = false;
                    pathState = PathState.SHOOT_TO_BAR_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;

            // ---------- BAR CYCLES LAST ----------
            case SHOOT_TO_BAR_FOLLOW:
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    setStoppersClosed(true);
                    follower.followPath(shootToBar, true);
                    pathStarted = true;
                    pathTimer.resetTimer();
                }

                if (follower.isBusy()) {
                    setStoppersClosed(true);
                    feederCmd = FEEDER_COLLECT_POWER;
                    intakeCmd = INTAKE_COLLECT_POWER;
                } else {
                    pathStarted = false;
                    barWaitStartSec = nowSec;
                    pathState = PathState.AT_BAR_INTAKE_WAIT;
                    pathTimer.resetTimer();
                }
                break;

            case AT_BAR_INTAKE_WAIT:
                commandShooterRpm(0.0, nowSec, dt);

                setStoppersClosed(true);
                feederCmd = FEEDER_COLLECT_POWER;
                intakeCmd = INTAKE_COLLECT_POWER;

                if (nowSec - barWaitStartSec >= BAR_INTAKE_TIME_SEC) {
                    feederCmd = 0.0;
                    intakeCmd = 0.0;
                    setStoppersClosed(false);

                    pathStarted = false;
                    enterSpinupState(nowSec);
                    pathState = PathState.BAR_TO_SHOOT_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;

            case BAR_TO_SHOOT_FOLLOW:
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    follower.followPath(barToShoot, true);
                    pathStarted = true;
                    pathState = PathState.SPINUP_BAR;
                    pathTimer.resetTimer();
                }
                break;

            case SPINUP_BAR: {
                setStoppersClosed(true);

                double dist = distToPoseIn(follower.getPose(), shootPose);
                if (!preSpinStarted && dist <= PRESPIN_DISTANCE_IN) {
                    preSpinStarted = true;
                    preSpinStartSec = nowSec;
                }

                if (preSpinStarted) {
                    commandShooterRpm(FAR_SHOOT_TARGET_RPM, nowSec, dt);
                    double rpmMeas = getShooterRpmMeasured();
                    shooterReadyCount = (Math.abs(rpmMeas - FAR_SHOOT_TARGET_RPM) <= SHOOT_READY_TOL_RPM)
                            ? (shooterReadyCount + 1) : 0;
                } else {
                    commandShooterRpm(0.0, nowSec, dt);
                    shooterReadyCount = 0;
                }

                if (!follower.isBusy()) {
                    boolean ready = shooterReadyCount >= SHOOT_READY_STABLE_CYCLES;
                    double spinupElapsed = (preSpinStartSec > 0.0) ? (nowSec - preSpinStartSec) : 0.0;
                    boolean timedOut = preSpinStarted && (spinupElapsed >= SPINUP_TIMEOUT_SEC);

                    if (ready || timedOut) {
                        enterShootingState(nowSec);
                        pathState = PathState.SHOOTING_BAR;
                        pathTimer.resetTimer();
                    }
                }
                break;
            }

            case SHOOTING_BAR:
                setStoppersClosed(false);
                commandShooterRpm(FAR_SHOOT_TARGET_RPM, nowSec, dt);

                updatePulse(nowSec);
                if (pulseOn) {
                    feederCmd = FEEDER_PULSE_POWER;
                    intakeCmd = INTAKE_PULSE_POWER;
                }

                if (nowSec - shootingStartSec >= SHOOT_DURATION_SEC) {
                    stopPulse();
                    feederCmd = 0.0;
                    intakeCmd = 0.0;

                    startShooterBrake(nowSec);
                    pathState = PathState.BRAKING_BAR;
                    pathTimer.resetTimer();
                }
                break;

            case BRAKING_BAR:
                setStoppersClosed(true);
                updateShooterBrake(nowSec);

                if (!shooterBrakeActive) {
                    barCycleCount++;

                    if (barCycleCount < BAR_CYCLES) {
                        pathStarted = false;
                        pathState = PathState.SHOOT_TO_BAR_FOLLOW;
                        pathTimer.resetTimer();
                    } else {
                        pathStarted = false;
                        pathState = PathState.SHOOT_TO_END_FOLLOW;
                        pathTimer.resetTimer();
                    }
                }
                break;

            // ---------- END ----------
            case SHOOT_TO_END_FOLLOW:
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);

                if (!pathStarted) {
                    follower.followPath(shootToEnd, true);
                    pathStarted = true;
                    pathTimer.resetTimer();
                }

                if (!follower.isBusy()) {
                    pathState = PathState.DONE;
                    pathTimer.resetTimer();
                }
                break;

            case DONE:
            default:
                setStoppersClosed(true);
                commandShooterRpm(0.0, nowSec, dt);
                break;
        }

        intakeMotor.setPower(Range.clip(intakeCmd, -1.0, 1.0));
        feederMotor.setPower(Range.clip(feederCmd, -1.0, 1.0));

        telemetry.addData("state", pathState);
        telemetry.addData("barCycle", "%d/%d", barCycleCount, BAR_CYCLES);
        telemetry.addData("pose x", "%.2f", follower.getPose().getX());
        telemetry.addData("pose y", "%.2f", follower.getPose().getY());
        telemetry.addData("heading(deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("stoppersClosed", stoppersClosed);
        telemetry.addData("pulseOn", pulseOn);
        telemetry.addData("rpm meas", "%.0f", getShooterRpmMeasured());
        telemetry.addData("pwr cmd", "%.3f", shooterPowerCmd);
        telemetry.addData("readyCount", shooterReadyCount);
        telemetry.addData("kickActive", shooterKickActive);
        telemetry.addData("brakeActive", shooterBrakeActive);
        telemetry.addData("stateTime(s)", "%.2f", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    // ===================== STOPPERS =====================
    private void setStoppersClosed(boolean closed) {
        if (closed == stoppersClosed) return;
        stoppersClosed = closed;

        if (closed) {
            leftStop.setPosition(LEFT_STOP_ZERO_POS);
            rightStop.setPosition(RIGHT_STOP_LEFT_POS);
        } else {
            leftStop.setPosition(LEFT_STOP_LEFT_POS);
            rightStop.setPosition(RIGHT_STOP_RIGHT_POS);
        }
    }

    // ===================== PULSE FEED =====================
    private void startPulse(double nowSec) {
        pulseOn = true;
        nextPulseToggleSec = nowSec + PULSE_ON_SEC;
    }

    private void stopPulse() {
        pulseOn = false;
        nextPulseToggleSec = 0.0;
    }

    private void updatePulse(double nowSec) {
        if (nextPulseToggleSec <= 0.0) return;
        if (nowSec < nextPulseToggleSec) return;

        pulseOn = !pulseOn;
        nextPulseToggleSec = nowSec + (pulseOn ? PULSE_ON_SEC : PULSE_OFF_SEC);
    }

    // ===================== UTILS =====================
    private static double distToPoseIn(Pose a, Pose b) {
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    // ===================== SHOOTER HELPERS (CUSTOM PIDF-IN-POWER) =====================
    private void resetShooterController() {
        shooterIntegral = 0.0;
        shooterLastError = 0.0;
    }

    private static double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * SHOOTER_TICKS_PER_REV;
    }

    private void commandShooterRpm(double rpm, double nowSec, double dt) {
        rpm = Math.max(0.0, rpm);

        if (rpm <= 1.0) {
            lastShooterCmdRpm = 0.0;
            shooterKickActive = false;
            shooterPowerCmd = 0.0;
            resetShooterController();

            if (!shooterBrakeActive) {
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterMotor.setPower(0.0);
            }
            return;
        }

        // any nonzero cancels braking
        shooterBrakeActive = false;

        // kick on 0 -> >0
        if (!shooterKickActive && lastShooterCmdRpm <= 1.0) {
            shooterKickActive = true;
            shooterKickEndSec = nowSec + KICK_TIME_SEC;
            resetShooterController();
        }

        lastShooterCmdRpm = rpm;

        if (shooterKickActive && nowSec < shooterKickEndSec) {
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterPowerCmd = Range.clip(KICK_POWER, 0.0, 1.0);
            shooterMotor.setPower(shooterPowerCmd);
            return;
        }

        shooterKickActive = false;

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetTPS = rpmToTicksPerSecond(rpm);
        double measuredTPS = shooterMotor.getVelocity();
        double error = targetTPS - measuredTPS;

        double fTerm = kF * targetTPS;
        double pTerm = kP * error;
        double dTerm = kD * ((error - shooterLastError) / Math.max(1e-3, dt));
        double prelim = fTerm + pTerm + dTerm;

        boolean wouldSaturateHigh = prelim >= 1.0;
        boolean wouldSaturateLow  = prelim <= 0.0;

        boolean allowIntegrate =
                (!wouldSaturateHigh && !wouldSaturateLow)
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
    }

    private double getShooterRpmMeasured() {
        double ticksPerSec = shooterMotor.getVelocity();
        return ticksPerSec * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    // ===================== HARSH BRAKE =====================
    private void startShooterBrake(double nowSec) {
        shooterKickActive = false;
        lastShooterCmdRpm = 0.0;
        resetShooterController();

        double rpm = getShooterRpmMeasured();
        double rpmAbs = Math.abs(rpm);

        if (rpmAbs < SHOOTER_BRAKE_STOP_RPM) {
            shooterBrakeActive = false;
            shooterPowerCmd = 0.0;
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
        if (!shooterBrakeActive) return;

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
        shooterPowerCmd = brakePower;
        shooterMotor.setPower(brakePower);
    }

    private void stopShooterBrake() {
        shooterBrakeActive = false;
        shooterPowerCmd = 0.0;
        shooterMotor.setPower(0.0);
    }
}
