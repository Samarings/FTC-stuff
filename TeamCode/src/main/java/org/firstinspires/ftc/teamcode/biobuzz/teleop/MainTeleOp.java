package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.ArrayDeque;
import java.util.List;

/**
 * Cheetahs 25920 BIOBUZZ single-file TeleOp.
 * Uses Pedro Constants/Tuning unmodified. No old BIOBUZZ helpers or autonomous modes.
 * G2 left stick: manual turret; stick press: auto/ manual.
 * Turret home = encoder count at OpMode INIT; allowed range +/-190 degrees.
 * WARNING: software-only limits cannot protect wiring if startup physical orientation is unsafe.
 * DO NOT run first tests with balls loaded. Confirm the drive gear ratio and sign on your machine.
 */
@Configurable
@TeleOp(name="BIOBUZZ Main TeleOp", group="BIOBUZZ")
public class MainTeleOp extends OpMode {
    // Hardware configuration names
    public static String TURRET_NAME="turret", LIMELIGHT_NAME="limelight";
    public static String SHOOTER_NAME="shooter", FEEDER_NAME="feeder", INTAKE_NAME="intake";
    public static boolean INTAKE_REVERSED=true, FEEDER_REVERSED=true, SHOOTER_REVERSED=false;
    public static double TURRET_AXIS_OFFSET_FWD_IN=0.0; // measure real robot-to-turret-axis offset
    public static double TURRET_AXIS_OFFSET_LEFT_IN=0.0; // measure real robot-to-turret-axis offset
    public static double SLOW_SCALE=0.4, TURN_SCALE=0.8;
    public static double FEEDER_MAX_POWER=1.0, INTAKE_MAX_POWER=1.0;
    public static double EJECT_POWER=0.7;
    public static double SHOOTER_TICKS_PER_REV=28.0; // keep original SHOOTER motor (not REV turret)
    public static double SHOOTER_MAX_RPM=3500.0, SHOOTER_IDLE_RPM=1200.0;
    public static double SHOOTER_RAMP_PER_SEC=3.0, SHOOTER_KICK_SEC=0.15;
    public static double SHOOTER_P=0.02, SHOOTER_I=0.0, SHOOTER_D=0.0001, SHOOTER_F=0.00042214;
    public static double TURRET_P=0.018, TURRET_I=0.0, TURRET_D=0.0008, TURRET_F=0.0;
    // Shooter RPM distance model: RPM(d) = RPM_REFERENCE * sqrt(d / DISTANCE_REFERENCE_IN).
    // One known calibration shot, NOT two near/far calibration points.
    public static double DISTANCE_REFERENCE_IN = 72.0;
    public static double RPM_REFERENCE = 2300.0;
    public static double RPM_TRIM_STEP=50.0;
    public static int AUTO_AIM_DIRECTION=-1, TURRET_ENCODER_SIGN=1;
    public static double TAG_ORDER_SIGN=1.0;
    // Match-specific tag cluster: red audience 34-37, red opposite 30-33;
    // blue audience 38-41, blue opposite 42-45.
    private boolean blue=false, audience=true, slow=false, shooterOn=false, idleOn=true, headingHoldOn=true;
    private boolean prevG2Stick=false, prevG2B=false, prevG2Back=false, prevG2Y=false, prevG2X=false;
    private boolean prevG2RightStick=false, prevG2L=false, prevG2R=false, prevG1X=false, prevG1Y=false;
    private boolean prevG1Lb=false, prevG1Rb=false, prevG1Up=false, prevG1Down=false, prevG1Left=false, prevG1Right=false;
    private double rpmTrim=0, lastSec=0, lastShooterError=0, shooterIntegral=0, shooterPower=0;
    private double kickUntil=-1, headingTarget=0, headingI=0, headingPrevErr=0;
    private double shooterTarget=0, shooterMeasured=0;
    private int tuningGroup=0, tuningIndex=0;
    private double tuningScale=1.0;
    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor feeder, intake;
    private StartupTurretController turretControl;

    @Override public void init() {
        follower=Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        shooter=hardwareMap.get(DcMotorEx.class,SHOOTER_NAME);
        feeder=hardwareMap.get(DcMotor.class,FEEDER_NAME);
        intake=hardwareMap.get(DcMotor.class,INTAKE_NAME);
        shooter.setDirection(SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setDirection(FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setPower(0); feeder.setPower(0); intake.setPower(0);
        turretControl=new StartupTurretController(hardwareMap);
        turretControl.setDirectionSign(TURRET_ENCODER_SIGN);
        StartupTurretController.TAG_ORDER_SIGN=TAG_ORDER_SIGN;
        turretControl.selectCluster(blue,audience);
        telemetry.addLine("INIT: startup position = 0 deg. Keep turret in SAFE cable orientation.");
        telemetry.addLine("REV UP: 5:1 * 3:1 -> ~396 gearbox RPM. goBILDA turret ~144 RPM.");
        telemetry.addLine("G1 X = BLUE, B = RED; G1 dpad up/down = audience/opposite during INIT");
        telemetry.addLine("No mechanisms move before START"); telemetry.update();
    }
    @Override public void init_loop() {
        if(gamepad1.x)blue=true;
        if(gamepad1.b)blue=false;
        if(gamepad1.dpad_up)audience=true;
        if(gamepad1.dpad_down)audience=false;
        turretControl.selectCluster(blue,audience);
        follower.updatePose();
        telemetry.addData("Alliance",blue?"BLUE":"RED");
        telemetry.addData("CELL",audience?"AUDIENCE":"OPPOSITE");
        telemetry.addData("Turret zero encoder", "at INIT"); telemetry.update();
    }
    @Override public void start(){ follower.startTeleopDrive(); lastSec=getRuntime(); headingTarget=follower.getPose().getHeading(); }
    @Override public void loop() {
        double now=getRuntime(), dt=Range.clip(now-lastSec,0.005,0.1); lastSec=now;
        // Update follower once per loop: current Pedro/Pinpoint pose is used by turret tracking.
        follower.update();
        Pose pose=follower.getPose();
        tuneControls();
        if(edge(gamepad1.y,prevG1Y))slow=!slow; prevG1Y=gamepad1.y;
        if(edge(gamepad1.x,prevG1X))headingHoldOn=!headingHoldOn; prevG1X=gamepad1.x;
        double scale=slow?SLOW_SCALE:1.0;
        double turn=-gamepad1.right_stick_x*TURN_SCALE*scale;
        if(headingHoldOn && Math.abs(turn)<0.04){
            double err=Math.atan2(Math.sin(headingTarget-pose.getHeading()),Math.cos(headingTarget-pose.getHeading()));
            headingI=Range.clip(headingI+err*dt,-1,1);
            turn=Range.clip(1.0*err+0.0*headingI+0.015*(err-headingPrevErr)/dt,-0.5,0.5);
            headingPrevErr=err;
        }else{headingTarget=pose.getHeading(); headingI=0; headingPrevErr=0;}
        follower.setTeleOpDrive(-gamepad1.left_stick_y*scale,-gamepad1.left_stick_x*scale,turn,false);
        if(edge(gamepad2.left_stick_button,prevG2Stick))turretControl.toggleAuto();prevG2Stick=gamepad2.left_stick_button;
        if(edge(gamepad2.right_stick_button,prevG2RightStick)){
            audience=!audience;turretControl.selectCluster(blue,audience);
        }prevG2RightStick=gamepad2.right_stick_button;
        if(edge(gamepad2.x,prevG2X)){AUTO_AIM_DIRECTION*=-1;turretControl.resetController();}prevG2X=gamepad2.x;
        if(edge(gamepad2.b,prevG2B))shooterOn=!shooterOn;prevG2B=gamepad2.b;
        if(edge(gamepad2.back,prevG2Back))idleOn=!idleOn;prevG2Back=gamepad2.back;
        if(edge(gamepad2.dpad_left,prevG2L))rpmTrim-=RPM_TRIM_STEP;prevG2L=gamepad2.dpad_left;
        if(edge(gamepad2.dpad_right,prevG2R))rpmTrim+=RPM_TRIM_STEP;prevG2R=gamepad2.dpad_right;
        StartupTurretController.KP=TURRET_P;StartupTurretController.KI=TURRET_I;
        StartupTurretController.KD=TURRET_D;StartupTurretController.KF=TURRET_F;
        StartupTurretController.TAG_ORDER_SIGN=TAG_ORDER_SIGN;
        turretControl.setDirectionSign(TURRET_ENCODER_SIGN);
        turretControl.update(gamepad2.left_stick_x,pose,now);
        // Dedicated flywheel PIDF. Shooter motor gearing remains UNCHANGED.
        double dist=turretControl.getDistanceIn();
        // The one mathematical RPM model. If range is unavailable, hold the calibrated
        // reference RPM (manual shooting fallback), not an invented distance estimate.
        double rpm=Double.isFinite(dist) && dist>0 ? calculateAutoRpm(dist)+rpmTrim : RPM_REFERENCE+rpmTrim;
        shooterTarget=shooterOn?Range.clip(rpm,0,SHOOTER_MAX_RPM):0;
        updateShooter(now,dt);
        double feederP=gamepad2.right_trigger>0.05?gamepad2.right_trigger*FEEDER_MAX_POWER:0;
        double intakeP=gamepad2.left_trigger>0.05?INTAKE_MAX_POWER:0;
        if(gamepad2.dpad_down){feederP=-EJECT_POWER;intakeP=-EJECT_POWER;}
        feeder.setPower(Range.clip(feederP,-1,1));intake.setPower(Range.clip(intakeP,-1,1));
        telemetry.addData("Turret",String.format("%s angle %.1f / limit +/-190; %s",turretControl.isAuto()?"AUTO":"MANUAL",turretControl.angleDeg(),turretControl.status()));
        telemetry.addData("REV turret ticks/deg",String.format("%.5f",StartupTurretController.TICKS_PER_DEGREE));
        telemetry.addData("AprilTags",String.format("raw %d, selected %d, ID %d, fresh %s",turretControl.getRawTagCount(),turretControl.getTagCount(),turretControl.getAcceptedTagId(),turretControl.isFrameFresh()));
        telemetry.addData("Center/error",String.format("%.1f in / %.2f deg",turretControl.getCenterXIn(),turretControl.getVisionErrorDeg()));
        telemetry.addData("Turret PIDF",String.format("P %.5f I %.5f D %.5f F %.5f",TURRET_P,TURRET_I,TURRET_D,TURRET_F));
        telemetry.addData("Shooter PIDF",String.format("P %.6f I %.6f D %.6f F %.6f",SHOOTER_P,SHOOTER_I,SHOOTER_D,SHOOTER_F));
        telemetry.addData("Shooter RPM target/actual",String.format("%.0f / %.0f",shooterTarget,shooterMeasured));
        telemetry.addData("Distance/RPM",String.format("%.1f in / %.0f rpm",dist,rpm));
        telemetry.addData("RPM model",String.format("RPM=%.0f*sqrt(d/%.0f); %s",RPM_REFERENCE,DISTANCE_REFERENCE_IN,
                Double.isFinite(dist)&&dist>0?"LL range":"reference fallback"));
        telemetry.addData("Tuning",tuningDescription());
        telemetry.addLine("G1 RB groups; LB step; Dpad up/down select, left/right adjust");
        telemetry.addLine("No two-minute auto-shutoff; Driver Station STOP ends TeleOp");
        telemetry.update();
    }
    private void updateShooter(double now,double dt){
        shooterMeasured=shooter.getVelocity()*60.0/SHOOTER_TICKS_PER_REV;
        if(shooterOn&&shooterTarget>0){
            if(kickUntil<0){kickUntil=now+SHOOTER_KICK_SEC;shooterIntegral=0;lastShooterError=0;}
            double ticksTarget=shooterTarget*SHOOTER_TICKS_PER_REV/60.0;
            double err=ticksTarget-shooter.getVelocity();
            if(now<kickUntil)shooterPower=1.0;
            else {
                double p=SHOOTER_F*ticksTarget+SHOOTER_P*err+SHOOTER_D*(err-lastShooterError)/dt;
                shooterIntegral=Range.clip(shooterIntegral+err*dt,-5000,5000);
                shooterPower=Range.clip(p+SHOOTER_I*shooterIntegral,0,1);
            }
            lastShooterError=err;
            // Invalid/negative encoder speed means we must not run a blind full-power loop.
            if(now>=kickUntil && shooterPower>0.6 && shooterMeasured<150){ shooterPower=0; }
        }else{
            kickUntil=-1; shooterIntegral=0;lastShooterError=0;
            double idle=idleOn?Range.clip(SHOOTER_F*SHOOTER_IDLE_RPM*SHOOTER_TICKS_PER_REV/60.0,0,1):0;
            double step=SHOOTER_RAMP_PER_SEC*dt;
            shooterPower+=Range.clip(idle-shooterPower,-step,step);
        }
        shooter.setPower(shooterPower);
    }
    public static double calculateAutoRpm(double distanceIn){
        if(!Double.isFinite(distanceIn) || distanceIn<=0 ||
                !Double.isFinite(DISTANCE_REFERENCE_IN) || DISTANCE_REFERENCE_IN<=0 ||
                !Double.isFinite(RPM_REFERENCE) || RPM_REFERENCE<0){
            return Double.NaN;
        }
        // Fixed-launch-angle first approximation; tune using measured shots.
        return Range.clip(RPM_REFERENCE*Math.sqrt(distanceIn/DISTANCE_REFERENCE_IN),0,SHOOTER_MAX_RPM);
    }
    private static boolean edge(boolean current,boolean previous){return current&&!previous;}
    private void tuneControls(){
        if(edge(gamepad1.right_bumper,prevG1Rb)){tuningGroup=(tuningGroup+1)%3;tuningIndex=0;}prevG1Rb=gamepad1.right_bumper;
        if(edge(gamepad1.left_bumper,prevG1Lb)){tuningScale=tuningScale>=10?0.1:tuningScale*10;}prevG1Lb=gamepad1.left_bumper;
        int count=tuningGroup==2?2:4;
        if(edge(gamepad1.dpad_up,prevG1Up))tuningIndex=(tuningIndex+count-1)%count;prevG1Up=gamepad1.dpad_up;
        if(edge(gamepad1.dpad_down,prevG1Down))tuningIndex=(tuningIndex+1)%count;prevG1Down=gamepad1.dpad_down;
        int dir=edge(gamepad1.dpad_right,prevG1Right)?1:edge(gamepad1.dpad_left,prevG1Left)?-1:0;
        prevG1Right=gamepad1.dpad_right;prevG1Left=gamepad1.dpad_left;
        if(dir==0)return;
        double[] tp={TURRET_P,TURRET_I,TURRET_D,TURRET_F};
        double[] sp={SHOOTER_P,SHOOTER_I,SHOOTER_D,SHOOTER_F};
        double[] curve={DISTANCE_REFERENCE_IN,RPM_REFERENCE};
        double[] arr=tuningGroup==0?tp:tuningGroup==1?sp:curve;
        double step=tuningGroup==0?new double[]{.001,.0001,.0001,.005}[tuningIndex]:tuningGroup==1?new double[]{.00001,.000001,.000001,.00001}[tuningIndex]:new double[]{1,25}[tuningIndex];
        arr[tuningIndex]=Math.max(0,arr[tuningIndex]+dir*step*tuningScale);
        if(tuningGroup==0){TURRET_P=arr[0];TURRET_I=arr[1];TURRET_D=arr[2];TURRET_F=arr[3];turretControl.resetController();}
        else if(tuningGroup==1){SHOOTER_P=arr[0];SHOOTER_I=arr[1];SHOOTER_D=arr[2];SHOOTER_F=arr[3];}
        else{DISTANCE_REFERENCE_IN=Math.max(1,arr[0]);RPM_REFERENCE=Range.clip(arr[1],0,SHOOTER_MAX_RPM);}
    }
    private String tuningDescription(){
        String g=tuningGroup==0?"TURRET":tuningGroup==1?"SHOOTER":"RPM MODEL";
        String n=tuningGroup==2?new String[]{"reference distance","reference RPM"}[tuningIndex]:new String[]{"P","I","D","F"}[tuningIndex];
        return g+" ["+n+"] step x"+tuningScale;
    }
    @Override public void stop(){
        if(turretControl!=null)turretControl.stop();
        if(shooter!=null)shooter.setPower(0);
        if(feeder!=null)feeder.setPower(0);
        if(intake!=null)intake.setPower(0);
        if(follower!=null){follower.setTeleOpDrive(0,0,0,false);}
    }

    private static final class StartupTurretController {

        // REV UltraPlanetary HD Hex 6000 RPM with 5:1 + 3:1 cartridges
        // ~396 RPM gearbox output; 2.75:1 goBILDA turret drive (~144 turret RPM no load).
        public static final double TICKS_PER_DEGREE =
                (28.0 * (68.0 / 13.0) * (84.0 / 29.0) * 2.75) / 360.0;

        // UPDATED: +/-190 degrees
        public static final double LIMIT_DEGREES = 190.0;

        // Begin slowing ahead of travel limits
        private static final double SOFT_ZONE_DEG = 25.0;
        private static final double LIMIT_LOOKAHEAD_S = 0.12;

        // No general automatic motor power cap,
        // but travel limit slowdown remains.
        public static double KP = 0.018;
        public static double KI = 0.0;
        public static double KD = 0.0008;
        public static double KF = 0.0;

        public static final double MANUAL_POWER = 0.35;

        private static final double DEADBAND_DEG = 0.8;
        private static final double MAX_FRAME_AGE_MS = 350.0;

        // Odometry fallback timeout
        public static double ODOM_HOLD_SEC = 5.0;

        private static final double TARGET_ALPHA = 0.35;

        private static final double CAMERA_FWD_IN =
                0.094 * 39.3700787;

        // Tag centers along the selected four-tag strip
        private static final double[] MEMBER_X_IN = {
                -6.5, -2.75, 2.75, 6.5
        };

        public static double TAG_ORDER_SIGN = 1.0;
        public static double AIM_TRIM_DEG = 0.0;

        private final DcMotorEx motor;
        private final Limelight3A limelight;

        private final int zeroTicks;
        private int directionSign = 1;

        private boolean auto = false;
        private boolean audienceSide = true;
        private int firstTag = 34;

        private double targetDeg = 0.0;
        private double integral = 0.0;
        private double lastError = 0.0;

        private double lastS = Double.NaN;
        private double lastAngle = Double.NaN;
        private double rateDegS = 0.0;

        private double lastSeenS =
                Double.NEGATIVE_INFINITY;

        private double distanceIn = Double.NaN;
        private double visionErrorDeg = Double.NaN;

        // Remembered target in Pinpoint coordinate frame
        private double targetX = Double.NaN;
        private double targetY = Double.NaN;

        private int tagCount = 0;
        private int rawTagCount = 0;
        private int acceptedTagId = -1;

        private double estimatedCenterXIn =
                Double.NaN;

        private boolean frameFresh = false;
        private String state = "MANUAL";

        // Pose and turret history for Limelight latency
        private static final class Sample {

            double t;
            double x;
            double y;
            double heading;
            double turret;

            Sample(
                    double t,
                    double x,
                    double y,
                    double heading,
                    double turret
            ) {
                this.t = t;
                this.x = x;
                this.y = y;
                this.heading = heading;
                this.turret = turret;
            }
        }

        private final ArrayDeque<Sample> history =
                new ArrayDeque<>();

        StartupTurretController(HardwareMap hw) {

            motor = hw.get(
                    DcMotorEx.class,
                    TURRET_NAME
            );

            limelight = hw.get(
                    Limelight3A.class,
                    LIMELIGHT_NAME
            );

            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );

            motor.setMode(
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
            );

            // Current encoder position becomes home
            zeroTicks = motor.getCurrentPosition();

            limelight.pipelineSwitch(0);
            limelight.start();
        }

        public void setDirectionSign(int sign) {
            directionSign = sign >= 0 ? 1 : -1;
        }

        public boolean isAuto() {
            return auto;
        }

        public int getFirstTag() {
            return firstTag;
        }

        public double angleDeg() {
            return directionSign *
                    (motor.getCurrentPosition() - zeroTicks)
                    / TICKS_PER_DEGREE;
        }

        public double getDistanceIn() {
            return distanceIn;
        }

        public double getVisionErrorDeg() {
            return visionErrorDeg;
        }

        public int getTagCount() {
            return tagCount;
        }

        public int getRawTagCount() {
            return rawTagCount;
        }

        public int getAcceptedTagId() {
            return acceptedTagId;
        }

        public boolean isFrameFresh() {
            return frameFresh;
        }

        public double getCenterXIn() {
            return estimatedCenterXIn;
        }

        public String status() {
            return state;
        }

        public double getTargetDeg() {
            return targetDeg;
        }

        public double getLastFixAge(double now) {
            return Double.isFinite(lastSeenS)
                    ? now - lastSeenS
                    : Double.NaN;
        }

        public boolean isOdomTracking(double now) {

            return auto
                    && Double.isFinite(targetX)
                    && Double.isFinite(targetY)
                    && now - lastSeenS <= ODOM_HOLD_SEC
                    && tagCount == 0;
        }

        public void resetController() {
            resetPid();
        }

        public void toggleAuto() {

            auto = !auto;
            targetDeg = angleDeg();
            resetPid();
        }

        public void clearRememberedTarget() {
            targetX = Double.NaN;
            targetY = Double.NaN;
            lastSeenS = Double.NEGATIVE_INFINITY;
            distanceIn = Double.NaN;
            history.clear();
            resetPid();
        }

        public void selectCluster(
                boolean blue,
                boolean audience
        ) {

            audienceSide = audience;

            firstTag =
                    blue
                            ? (audience ? 38 : 42)
                            : (audience ? 34 : 30);

            clearRememberedTarget();
            tagCount = 0;
        }

        private void resetPid() {
            integral = 0.0;
            lastError = 0.0;
            lastS = Double.NaN;
        }

        private static double clip(
                double x,
                double lo,
                double hi
        ) {
            return Math.max(lo, Math.min(hi, x));
        }

        private static double wrapDeg(double degrees) {

            return ((degrees + 180.0) % 360.0 + 360.0)
                    % 360.0 - 180.0;
        }

        private static double wrapRad(double radians) {

            return Math.atan2(
                    Math.sin(radians),
                    Math.cos(radians)
            );
        }

        private Sample interpolated(double time) {

            if (history.isEmpty()) {
                return null;
            }

            Sample prev = null;

            for (Sample s : history) {

                if (s.t >= time) {

                    if (prev == null) {
                        return s;
                    }

                    double fraction = clip(
                            (time - prev.t) /
                                    Math.max(
                                            1e-6,
                                            s.t - prev.t
                                    ),
                            0.0,
                            1.0
                    );

                    return new Sample(
                            time,
                            prev.x +
                                    fraction * (s.x - prev.x),
                            prev.y +
                                    fraction * (s.y - prev.y),
                            prev.heading +
                                    fraction * wrapRad(
                                            s.heading - prev.heading
                                    ),
                            prev.turret +
                                    fraction * (s.turret - prev.turret)
                    );
                }

                prev = s;
            }

            return history.peekLast();
        }

        private void samplePose(
                Pose pose,
                double turretDeg,
                double now
        ) {

            if (pose == null) {
                return;
            }

            if (!Double.isFinite(pose.getX())
                    || !Double.isFinite(pose.getY())
                    || !Double.isFinite(pose.getHeading())) {
                return;
            }

            history.addLast(
                    new Sample(
                            now,
                            pose.getX(),
                            pose.getY(),
                            pose.getHeading(),
                            turretDeg
                    )
            );

            while (!history.isEmpty()
                    && now - history.peekFirst().t > 1.0) {

                history.removeFirst();
            }
        }

        private static double[] axisXY(
                double x,
                double y,
                double heading
        ) {

            double forward =
                    TURRET_AXIS_OFFSET_FWD_IN;

            double left =
                    TURRET_AXIS_OFFSET_LEFT_IN;

            return new double[]{
                    x + forward * Math.cos(heading)
                            - left * Math.sin(heading),

                    y + forward * Math.sin(heading)
                            + left * Math.cos(heading)
            };
        }

        // =====================================================
        // TURRET UPDATE
        // =====================================================

        public void update(
                double manualStickX,
                Pose pose,
                double now
        ) {

            final double angle = angleDeg();

            double dt =
                    Double.isNaN(lastS)
                            ? 0.02
                            : clip(
                            now - lastS,
                            0.005,
                            0.1
                    );

            rateDegS =
                    Double.isNaN(lastAngle)
                            ? 0.0
                            : (angle - lastAngle) / dt;

            lastS = now;
            lastAngle = angle;

            samplePose(pose, angle, now);
            observe(now);

            // ---------------- MANUAL
            if (!auto) {

                double power =
                        Math.abs(manualStickX) < 0.06
                                ? 0.0
                                : -manualStickX
                                * MANUAL_POWER
                                * directionSign;

                applyLimitedPower(power, angle);

                targetDeg = angle;

                state = "MANUAL";

                return;
            }

            // ---------------- AUTO

            boolean targetStored =
                    Double.isFinite(targetX)
                            && Double.isFinite(targetY);

            boolean useful =
                    targetStored
                            && now - lastSeenS <= ODOM_HOLD_SEC;

            if (useful
                    && pose != null
                    && Double.isFinite(pose.getX())
                    && Double.isFinite(pose.getY())
                    && Double.isFinite(pose.getHeading())) {

                double[] axis = axisXY(
                        pose.getX(),
                        pose.getY(),
                        pose.getHeading()
                );

                double dx = targetX - axis[0];
                double dy = targetY - axis[1];

                distanceIn = Math.hypot(dx, dy);

                // Target bearing relative to chassis heading
                double desired = wrapDeg(
                        Math.toDegrees(
                                Math.atan2(dy, dx)
                                        - pose.getHeading()
                        )
                );

                targetDeg = chooseEquivalent(
                        angle,
                        desired
                );

                state =
                        tagCount > 0
                                ? "VISION LOCK "
                                + tagCount + " TAG(S)"
                                : String.format(
                                "ODOM HOLD %.1fs",
                                now - lastSeenS
                        );

            } else {

                // Don't continue tracking an expired target
                targetDeg = angle;
                integral = 0.0;

                state =
                        !targetStored
                                ? "WAIT FOR TARGET"
                                : "ODOM EXPIRED - HOLD";

                distanceIn = Double.NaN;
            }

            double error = targetDeg - angle;

            if (Math.abs(error) < DEADBAND_DEG) {

                applyLimitedPower(0.0, angle);

                lastError = error;
                integral = 0.0;

                return;
            }

            integral = clip(
                    integral + error * dt,
                    -40.0,
                    40.0
            );

            double derivative =
                    (error - lastError) / dt;

            lastError = error;

            // Position PIDF
            double power =
                    KP * error
                            + KI * integral
                            + KD * derivative
                            + KF * Math.signum(error);

            // Full available motor power
            applyLimitedPower(
                    clip(power, -1.0, 1.0) * directionSign,
                    angle
            );
        }

        // =====================================================
        // TRAVEL LIMIT PROTECTION
        // =====================================================

        private void applyLimitedPower(
                double motorPower,
                double angle
        ) {

            double logical =
                    motorPower * directionSign;

            // Predict near-future encoder angle
            double forecast =
                    angle + rateDegS * LIMIT_LOOKAHEAD_S;

            if (logical > 0.0) {

                if (angle >= LIMIT_DEGREES) {

                    logical = 0.0;

                } else {

                    logical *= clip(
                            (LIMIT_DEGREES -
                                    Math.max(angle, forecast))
                                    / SOFT_ZONE_DEG,
                            0.0,
                            1.0
                    );
                }

            } else if (logical < 0.0) {

                if (angle <= -LIMIT_DEGREES) {

                    logical = 0.0;

                } else {

                    logical *= clip(
                            (LIMIT_DEGREES +
                                    Math.min(angle, forecast))
                                    / SOFT_ZONE_DEG,
                            0.0,
                            1.0
                    );
                }
            }

            motor.setPower(
                    clip(
                            logical * directionSign,
                            -1.0,
                            1.0
                    )
            );
        }

        // =====================================================
        // APRILTAG DETECTION
        // =====================================================

        private void observe(double now) {

            tagCount = 0;
            rawTagCount = 0;
            acceptedTagId = -1;

            frameFresh = false;
            estimatedCenterXIn = Double.NaN;
            visionErrorDeg = Double.NaN;

            LLResult result =
                    limelight.getLatestResult();

            if (result == null
                    || result.getStaleness()
                    > MAX_FRAME_AGE_MS) {
                return;
            }

            frameFresh = true;

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            if (tags == null || tags.isEmpty()) {
                return;
            }

            rawTagCount = tags.size();

            double sumX = 0.0;
            double sumZ = 0.0;
            double sumW = 0.0;
            double best = -1.0;

            for (LLResultTypes.FiducialResult tag : tags) {

                int slot =
                        tag.getFiducialId() - firstTag;

                // Only selected four-tag cluster
                if (slot < 0 || slot >= 4) {
                    continue;
                }

                if (!Double.isFinite(
                        tag.getTargetXDegrees()
                )) {
                    continue;
                }

                try {

                    Position p =
                            tag.getTargetPoseCameraSpace()
                                    .getPosition()
                                    .toUnit(DistanceUnit.INCH);

                    double z = Math.abs(p.z);

                    if (!Double.isFinite(z)
                            || z < 10.0
                            || z > 350.0) {
                        continue;
                    }

                    double x =
                            z * Math.tan(
                                    Math.toRadians(
                                            tag.getTargetXDegrees()
                                    )
                            );

                    double centerX =
                            x - TAG_ORDER_SIGN
                                    * MEMBER_X_IN[slot];

                    double weight =
                            Math.max(
                                    0.05,
                                    tag.getTargetArea()
                            );

                    if (!Double.isFinite(centerX)) {
                        continue;
                    }

                    sumX += centerX * weight;
                    sumZ += z * weight;
                    sumW += weight;

                    tagCount++;

                    if (weight > best) {
                        best = weight;
                        acceptedTagId =
                                tag.getFiducialId();
                    }

                } catch (RuntimeException ignored) {
                    // Skip malformed tag pose
                }
            }

            if (tagCount == 0 || sumW <= 0.0) {
                return;
            }

            double cx = sumX / sumW;
            double cz = sumZ / sumW;

            estimatedCenterXIn = cx;

            visionErrorDeg =
                    Math.toDegrees(
                            Math.atan2(cx, cz)
                    ) + AIM_TRIM_DEG;

            // Confirmed reversed aiming direction
            double correction =
                    -AUTO_AIM_DIRECTION
                            * visionErrorDeg;

            double range =
                    Math.hypot(
                            cx,
                            cz + CAMERA_FWD_IN
                    );

            // Compensate for camera latency
            double latencyMs =
                    Math.max(
                            0.0,
                            result.getStaleness()
                    )
                            + Math.max(
                            0.0,
                            result.getCaptureLatency()
                    )
                            + Math.max(
                            0.0,
                            result.getTargetingLatency()
                    );

            Sample atCapture =
                    interpolated(
                            now - latencyMs / 1000.0
                    );

            if (atCapture == null) {
                return;
            }

            double[] axis = axisXY(
                    atCapture.x,
                    atCapture.y,
                    atCapture.heading
            );

            double absolute =
                    atCapture.heading
                            + Math.toRadians(
                            atCapture.turret + correction
                    );

            double mx =
                    axis[0]
                            + range * Math.cos(absolute);

            double my =
                    axis[1]
                            + range * Math.sin(absolute);

            if (!Double.isFinite(mx)
                    || !Double.isFinite(my)) {
                return;
            }

            if (!Double.isFinite(targetX)
                    || now - lastSeenS > ODOM_HOLD_SEC) {

                targetX = mx;
                targetY = my;

            } else {

                targetX += TARGET_ALPHA
                        * (mx - targetX);

                targetY += TARGET_ALPHA
                        * (my - targetY);
            }

            lastSeenS = now;
        }

        // =====================================================
        // LEGAL ANGLE SELECTION: +/-190 DEGREES
        // =====================================================

        private static double chooseEquivalent(
                double current,
                double desired
        ) {

            double best = clip(
                    desired,
                    -LIMIT_DEGREES,
                    LIMIT_DEGREES
            );

            double cost = Double.POSITIVE_INFINITY;

            for (int k = -2; k <= 2; k++) {

                double candidate =
                        desired + 360.0 * k;

                if (candidate < -LIMIT_DEGREES
                        || candidate > LIMIT_DEGREES) {
                    continue;
                }

                double d =
                        Math.abs(candidate - current);

                if (d < cost) {
                    cost = d;
                    best = candidate;
                }
            }

            return best;
        }

        public void stop() {
            motor.setPower(0.0);
        }
    }
}
