package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RedMain extends OpMode {

    // ===================== PEDRO =====================
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START_TO_SHOOT_FOLLOW,
        SPINUP_1,
        SHOOTING_1,
        BRAKING_1,

        SHOOT_TO_2NDSET_FOLLOW,
        SECONDSET_TO_FIRSTBARHIT_FOLLOW,
        FIRSTBARHIT_TO_SHOOT_FOLLOW,
        SPINUP_2,
        SHOOTING_2,
        BRAKING_2,

        // -------- BAR CYCLES (REPEAT) --------
        SHOOT_TO_BAR_FOLLOW,
        AT_BAR_INTAKE_WAIT,
        BAR_TO_SHOOT_FOLLOW,
        SPINUP_BAR,
        SHOOTING_BAR,
        BRAKING_BAR,

        // -------- 1ST SET CYCLE --------
        SHOOT_TO_1STSET_FOLLOW,
        FIRSTSET_TO_INTAKED_FIRSTSET_FOLLOW,
        INTAKED_FIRSTSET_TO_SHOOT_FOLLOW,
        SPINUP_3,
        SHOOTING_3,
        BRAKING_3,

        // -------- FINAL PARK/END --------
        SHOOT_TO_END_FOLLOW,

        DONE
    }

    private PathState pathState;

    // ===================== POSES (INCHES) =====================
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose shootPose = new Pose(-25.256, -47.187, Math.toRadians(49));

    private final Pose secondSetPose   = new Pose(-19.5, -66, Math.toRadians(0)); // <-- CHANGE ME
    private final Pose firstBarHitPose = new Pose(2,  -66, Math.toRadians(0));    // <-- CHANGE ME

    private final Pose barPose = new Pose(15.3, -66.8, Math.toRadians(22.18));   // <-- CHANGE ME

    // 1ST SET POSES
    private final Pose firstSetPose        = new Pose(-16, -43, Math.toRadians(0));  // <-- CHANGE ME
    private final Pose intakedFirstSetPose = new Pose(5.2, -43, Math.toRadians(0)); // <-- CHANGE ME

    // END POSE
    private final Pose endPose = new Pose(-20, -51, Math.toRadians(35)); // <-- CHANGE ME

    // ===================== BEZIER CONTROL POINTS (TUNE X/Y) =====================
    // These are NOT "midpoints" you stop at. They are control points that bend the curve.
    // Heading here does not affect the curve geometry; only X/Y matter for the curve shape.
    private final Pose shootToBarControl = new Pose(-13.5, 63.4, Math.toRadians(-49));      // <-- CHANGE X/Y
    private final Pose barToShootControl = new Pose(-13.5, 63.4, Math.toRadians(-22.18));  // <-- CHANGE X/Y

    // ===================== PATHS =====================
    private PathChain startToShoot;
    private PathChain shootToSecondSet;
    private PathChain secondSetToFirstBarHit;
    private PathChain firstBarHitToShoot;

    // Bezier CURVES for shoot<->bar (no "hold at midpoint")
    private PathChain shootToBar;
    private PathChain barToShoot;

    private PathChain shootTo1stSet;
    private PathChain firstSetToIntaked1stSet;
    private PathChain intaked1stSetToShoot;

    private PathChain shootToEnd;

    // ===================== HARDWARE =====================
    private DcMotor intakeMotor;
    private DcMotor feederMotor;
    private DcMotorEx shooterMotor;

    // ===================== STOPPERS =====================
    private Servo leftStop, rightStop;
    private boolean stoppersClosed = false; // start false so first call applies positions

    private static final double LEFT_STOP_ZERO_POS   = 0.50; // closed
    private static final double LEFT_STOP_LEFT_POS   = 0.00; // open
    private static final double RIGHT_STOP_LEFT_POS  = 0.00; // closed
    private static final double RIGHT_STOP_RIGHT_POS = 1.00; // open

    // ===================== SHOOTER CONTROL (CUSTOM POWER PIDF) =====================
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
    private static final double SHOOT_TARGET_RPM = 2350;
    private static final double SHOOT_READY_TOL_RPM = 100;
    private static final int    SHOOT_READY_STABLE_CYCLES = 5;

    private static final double PRESPIN_DISTANCE_IN = 30;
    private static final double SPINUP_TIMEOUT_SEC = 5;

    private static final double SHOOT_DURATION_SEC = 1.15;

    private static final double FEEDER_SHOOT_POWER = 1.0;
    private static final double INTAKE_SHOOT_POWER = 1.0;

    private static final double FEEDER_COLLECT_POWER = 1.0;
    private static final double INTAKE_COLLECT_POWER = 1.0;

    private static final double BAR_INTAKE_TIME_SEC = 1.2; // <-- adjust
    private static final int    BAR_CYCLES = 2;

    // ===================== KICK SPINUP =====================
    private static final double KICK_TIME_SEC = 0.15;
    private static final double KICK_POWER    = 1.0;

    private boolean shooterKickActive = false;
    private double  shooterKickEndSec = 0.0;
    private double  lastShooterCmdRpm = 0.0;

    // ===================== HARSH BRAKE =====================
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

    // BAR bookkeeping
    private int barCycleCount = 0;
    private double barWaitStartSec = -1.0;

    // ===================== PATH BUILD =====================
    private void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

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

        // ==========================================================
        // NEW: Bezier CURVES between shoot <-> bar using control points
        // This avoids “stop at midpoint” behavior from split segments.
        // ==========================================================
        shootToBar = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToBarControl, barPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), barPose.getHeading())
                .build();

        barToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(barPose, barToShootControl, shootPose))
                .setLinearHeadingInterpolation(barPose.getHeading(), shootPose.getHeading())
                .build();

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

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    private void enterSpinupState(double nowSec) {
        preSpinStarted = false;
        preSpinStartSec = -1.0;
        shooterReadyCount = 0;

        shooterBrakeActive = false;
        shooterKickActive = false;

        resetShooterController();
    }

    private void enterShootingState(double nowSec) {
        shootingStartSec = nowSec;
    }

    // ===================== INIT =====================
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

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

        pathState = PathState.START_TO_SHOOT_FOLLOW;
        pathStarted = false;

        barCycleCount = 0;
        barWaitStartSec = -1.0;

        telemetry.addLine("AutoTest: shoot<->bar now uses BezierCurve(control point) (no midpoint stopping).");
        telemetry.addLine("Tune shootToBarControl / barToShootControl X/Y to avoid obstacle.");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        lastLoopSec = getRuntime();
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

            // ---------- FIRST SHOOT SEQUENCE ----------
            case START_TO_SHOOT_FOLLOW:
                commandShooterRpm(0.0, nowSec, dt);
                setStoppersClosed(true);

                if (!pathStarted) {
                    follower.followPath(startToShoot, true);
                    pathStarted = true;

                    enterSpinupState(nowSec);
                    pathState = PathState.SPINUP_1;
                    pathTimer.resetTimer();
                }
                break;

            case SPINUP_1: {
                setStoppersClosed(true);

                double dist1 = distToPoseIn(follower.getPose(), shootPose);
                if (!preSpinStarted && dist1 <= PRESPIN_DISTANCE_IN) {
                    preSpinStarted = true;
                    preSpinStartSec = nowSec;
                }

                if (preSpinStarted) {
                    commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);
                    double rpmMeas = getShooterRpmMeasured();
                    if (Math.abs(rpmMeas - SHOOT_TARGET_RPM) <= SHOOT_READY_TOL_RPM) shooterReadyCount++;
                    else shooterReadyCount = 0;
                } else {
                    commandShooterRpm(0.0, nowSec, dt);
                    shooterReadyCount = 0;
                }

                if (!follower.isBusy()) {
                    boolean ready = shooterReadyCount >= SHOOT_READY_STABLE_CYCLES;
                    double spinupElapsed = (preSpinStartSec > 0.0) ? (nowSec - preSpinStartSec) : 0.0;
                    boolean timedOut = preSpinStarted && (spinupElapsed >= SPINUP_TIMEOUT_SEC);

                    if (!preSpinStarted) {
                        preSpinStarted = true;
                        preSpinStartSec = nowSec;
                        commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);
                        break;
                    }

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
                commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);

                feederCmd = FEEDER_SHOOT_POWER;
                intakeCmd = INTAKE_SHOOT_POWER;

                if (nowSec - shootingStartSec >= SHOOT_DURATION_SEC) {
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

            // ---------- MOVE TO 2ND SET ----------
            case SHOOT_TO_2NDSET_FOLLOW:
                commandShooterRpm(0.0, nowSec, dt);
                setStoppersClosed(true);

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

            // ---------- COLLECT: 2ND SET -> FIRST BAR HIT ----------
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
                    pathState = PathState.FIRSTBARHIT_TO_SHOOT_FOLLOW;
                    enterSpinupState(nowSec);
                    pathTimer.resetTimer();
                }
                break;

            // ---------- RETURN TO SHOOT ----------
            case FIRSTBARHIT_TO_SHOOT_FOLLOW:
                setStoppersClosed(true);
                if (!pathStarted) {
                    follower.followPath(firstBarHitToShoot, true);
                    pathStarted = true;
                    pathState = PathState.SPINUP_2;
                    pathTimer.resetTimer();
                }
                break;

            case SPINUP_2: {
                setStoppersClosed(true);

                double dist2 = distToPoseIn(follower.getPose(), shootPose);
                if (!preSpinStarted && dist2 <= PRESPIN_DISTANCE_IN) {
                    preSpinStarted = true;
                    preSpinStartSec = nowSec;
                }

                if (preSpinStarted) {
                    commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);
                    double rpmMeas = getShooterRpmMeasured();
                    if (Math.abs(rpmMeas - SHOOT_TARGET_RPM) <= SHOOT_READY_TOL_RPM) shooterReadyCount++;
                    else shooterReadyCount = 0;
                } else {
                    commandShooterRpm(0.0, nowSec, dt);
                    shooterReadyCount = 0;
                }

                if (!follower.isBusy()) {
                    boolean ready = shooterReadyCount >= SHOOT_READY_STABLE_CYCLES;
                    double spinupElapsed = (preSpinStartSec > 0.0) ? (nowSec - preSpinStartSec) : 0.0;
                    boolean timedOut = preSpinStarted && (spinupElapsed >= SPINUP_TIMEOUT_SEC);

                    if (!preSpinStarted) {
                        preSpinStarted = true;
                        preSpinStartSec = nowSec;
                        commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);
                        break;
                    }

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
                commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);

                feederCmd = FEEDER_SHOOT_POWER;
                intakeCmd = INTAKE_SHOOT_POWER;

                if (nowSec - shootingStartSec >= SHOOT_DURATION_SEC) {
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

            // ===================== BAR CYCLES =====================
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
                commandShooterRpm(0.0, nowSec, dt);
                setStoppersClosed(true);

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
                    commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);
                    double rpmMeas = getShooterRpmMeasured();
                    if (Math.abs(rpmMeas - SHOOT_TARGET_RPM) <= SHOOT_READY_TOL_RPM) shooterReadyCount++;
                    else shooterReadyCount = 0;
                } else {
                    commandShooterRpm(0.0, nowSec, dt);
                    shooterReadyCount = 0;
                }

                if (!follower.isBusy()) {
                    boolean ready = shooterReadyCount >= SHOOT_READY_STABLE_CYCLES;
                    double spinupElapsed = (preSpinStartSec > 0.0) ? (nowSec - preSpinStartSec) : 0.0;
                    boolean timedOut = preSpinStarted && (spinupElapsed >= SPINUP_TIMEOUT_SEC);

                    if (!preSpinStarted) {
                        preSpinStarted = true;
                        preSpinStartSec = nowSec;
                        commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);
                        break;
                    }

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
                commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);

                feederCmd = FEEDER_SHOOT_POWER;
                intakeCmd = INTAKE_SHOOT_POWER;

                if (nowSec - shootingStartSec >= SHOOT_DURATION_SEC) {
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
                        pathState = PathState.SHOOT_TO_1STSET_FOLLOW;
                        pathTimer.resetTimer();
                    }
                }
                break;

            // ===================== 1ST SET PATHS =====================
            case SHOOT_TO_1STSET_FOLLOW:
                commandShooterRpm(0.0, nowSec, dt);
                setStoppersClosed(true);

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
                    pathState = PathState.SPINUP_3;
                    pathTimer.resetTimer();
                }
                break;

            case SPINUP_3: {
                setStoppersClosed(true);

                double dist3 = distToPoseIn(follower.getPose(), shootPose);
                if (!preSpinStarted && dist3 <= PRESPIN_DISTANCE_IN) {
                    preSpinStarted = true;
                    preSpinStartSec = nowSec;
                }

                if (preSpinStarted) {
                    commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);
                    double rpmMeas = getShooterRpmMeasured();
                    if (Math.abs(rpmMeas - SHOOT_TARGET_RPM) <= SHOOT_READY_TOL_RPM) shooterReadyCount++;
                    else shooterReadyCount = 0;
                } else {
                    commandShooterRpm(0.0, nowSec, dt);
                    shooterReadyCount = 0;
                }

                if (!follower.isBusy()) {
                    boolean ready = shooterReadyCount >= SHOOT_READY_STABLE_CYCLES;
                    double spinupElapsed = (preSpinStartSec > 0.0) ? (nowSec - preSpinStartSec) : 0.0;
                    boolean timedOut = preSpinStarted && (spinupElapsed >= SPINUP_TIMEOUT_SEC);

                    if (!preSpinStarted) {
                        preSpinStarted = true;
                        preSpinStartSec = nowSec;
                        commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);
                        break;
                    }

                    if (ready || timedOut) {
                        enterShootingState(nowSec);
                        pathState = PathState.SHOOTING_3;
                        pathTimer.resetTimer();
                    }
                }
                break;
            }

            case SHOOTING_3:
                setStoppersClosed(false);
                commandShooterRpm(SHOOT_TARGET_RPM, nowSec, dt);

                feederCmd = FEEDER_SHOOT_POWER;
                intakeCmd = INTAKE_SHOOT_POWER;

                if (nowSec - shootingStartSec >= SHOOT_DURATION_SEC) {
                    feederCmd = 0.0;
                    intakeCmd = 0.0;
                    startShooterBrake(nowSec);
                    pathState = PathState.BRAKING_3;
                    pathTimer.resetTimer();
                }
                break;

            case BRAKING_3:
                setStoppersClosed(true);
                updateShooterBrake(nowSec);

                if (!shooterBrakeActive) {
                    pathStarted = false;
                    pathState = PathState.SHOOT_TO_END_FOLLOW;
                    pathTimer.resetTimer();
                }
                break;

            // ===================== FINAL END PATH =====================
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

        // Apply motors
        intakeMotor.setPower(Range.clip(intakeCmd, -1.0, 1.0));
        feederMotor.setPower(Range.clip(feederCmd, -1.0, 1.0));

        // Telemetry
        telemetry.addData("state", pathState);
        telemetry.addData("barCycle", "%d/%d", barCycleCount, BAR_CYCLES);
        telemetry.addData("pose x", "%.2f", follower.getPose().getX());
        telemetry.addData("pose y", "%.2f", follower.getPose().getY());
        telemetry.addData("heading(deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("dist->shoot(in)", "%.2f", distToPoseIn(follower.getPose(), shootPose));

        telemetry.addData("stoppersClosed", stoppersClosed);
        telemetry.addData("intakeCmd", "%.2f", intakeCmd);
        telemetry.addData("feederCmd", "%.2f", feederCmd);

        telemetry.addData("preSpin", preSpinStarted);
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

        // any nonzero command cancels braking
        shooterBrakeActive = false;

        // start kick when going from 0 -> >0
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
