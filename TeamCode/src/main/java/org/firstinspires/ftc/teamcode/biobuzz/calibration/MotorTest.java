package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.biobuzz.Buttons;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

/**
 * Check the intake, feeder, shooter, turret motor and stopper servos one at a time: names,
 * wiring, directions, encoders and current. Directions come from RobotConstants, so what you
 * see here is what the match code does.
 *
 * Gamepad 1:
 *   dpad up/down   select device
 *   left stick Y   power for the selected motor (up = +), or move the selected servo
 *   X / Y          stopper servos: CLOSED / OPEN positions from RobotConstants
 *   B              stop everything
 *
 * Expected: + intake/feeder power pulls balls IN toward the shooter (else flip
 * INTAKE_REVERSED / FEEDER_REVERSED); + shooter power launches forward and the RPM reads
 * positive (else flip SHOOTER_REVERSED). Shooter power is capped at RPM_CAP / 6000.
 * The turret motor is NEVER powered while TURRET_FIXED; only its encoder is shown.
 */
@TeleOp(name = "Motor Test (intake/feeder/shooter/turret)", group = "Calibration")
public class MotorTest extends OpMode {
    private static final String[] NAMES = {"intake", "feeder", "shooter", "turret", "leftStop", "rightStop"};

    private DcMotorEx intake, feeder, shooter, turretMotor;
    private Servo leftStop, rightStop;
    private Turret turret;
    private int sel = 0;
    private double leftPos = Double.NaN, rightPos = Double.NaN;
    private final Buttons buttons = new Buttons();
    private double lastSec = -1;

    @Override
    public void init() {
        intake = motor(INTAKE_NAME, INTAKE_REVERSED, DcMotor.ZeroPowerBehavior.BRAKE);
        feeder = motor(FEEDER_NAME, FEEDER_REVERSED, DcMotor.ZeroPowerBehavior.BRAKE);
        shooter = motor(SHOOTER_NAME, SHOOTER_REVERSED, DcMotor.ZeroPowerBehavior.FLOAT);
        turret = new Turret(hardwareMap, null); // handles direction, FLOAT when locked, limits
        try { turretMotor = hardwareMap.get(DcMotorEx.class, TURRET_NAME); } catch (Exception e) { turretMotor = null; }
        try { leftStop = hardwareMap.get(Servo.class, LEFT_STOP_NAME); } catch (Exception e) { leftStop = null; }
        try { rightStop = hardwareMap.get(Servo.class, RIGHT_STOP_NAME); } catch (Exception e) { rightStop = null; }
    }

    private DcMotorEx motor(String name, boolean reversed, DcMotor.ZeroPowerBehavior zpb) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(zpb);
            m.setPower(0.0);
            return m;
        } catch (Exception e) {
            return null;
        }
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Motor Test: press START. Nothing moves until you push a stick.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = lastSec < 0 ? 0.02 : Math.max(1e-3, now - lastSec);
        lastSec = now;

        if (buttons.pressed("up", gamepad1.dpad_up)) sel = (sel + NAMES.length - 1) % NAMES.length;
        if (buttons.pressed("dn", gamepad1.dpad_down)) sel = (sel + 1) % NAMES.length;
        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) < 0.05) stick = 0;
        boolean stopAll = gamepad1.b;

        set(intake, sel == 0 && !stopAll ? stick : 0);
        set(feeder, sel == 1 && !stopAll ? stick : 0);
        double shooterCap = rpmCap() / RPM_HARD_MAX;
        set(shooter, sel == 2 && !stopAll ? Range.clip(stick, -shooterCap, shooterCap) : 0);

        // Turret: only through the Turret class (limits, faults; 0 power while locked/uncalibrated).
        if (sel == 3 && !stopAll) turret.setManualPower(stick);
        else turret.off();
        turret.update(now);

        if (leftStop != null) {
            if (buttons.pressed("x", gamepad1.x)) { leftPos = LEFT_STOP_CLOSED; if (rightStop != null) rightPos = RIGHT_STOP_CLOSED; }
            if (buttons.pressed("y", gamepad1.y)) { leftPos = LEFT_STOP_OPEN; if (rightStop != null) rightPos = RIGHT_STOP_OPEN; }
        }
        if (sel == 4 && leftStop != null && stick != 0) leftPos = Range.clip((Double.isNaN(leftPos) ? 0.5 : leftPos) + stick * 0.5 * dt, 0, 1);
        if (sel == 5 && rightStop != null && stick != 0) rightPos = Range.clip((Double.isNaN(rightPos) ? 0.5 : rightPos) + stick * 0.5 * dt, 0, 1);
        if (leftStop != null && !Double.isNaN(leftPos)) leftStop.setPosition(leftPos);
        if (rightStop != null && !Double.isNaN(rightPos)) rightStop.setPosition(rightPos);

        telemetry.addLine("Selected (dpad): >> " + NAMES[sel] + " <<   stick=" + String.format("%.2f", stick));
        telemetry.addLine(line("intake", intake, 1));
        telemetry.addLine(line("feeder", feeder, 1));
        telemetry.addLine(line("shooter", shooter, 60.0 / SHOOTER_TICKS_PER_REV) + String.format("  (cap pwr %.2f)", shooterCap));
        telemetry.addLine(turret.describe() + (turretMotor != null ? String.format("  %.2fA", turretMotor.getCurrent(CurrentUnit.AMPS)) : ""));
        if (TURRET_FIXED) telemetry.addLine("  turret LOCKED (TURRET_FIXED): never powered, encoder only");
        else if (!Turret.isCalibrated()) telemetry.addLine("  turret not calibrated: never powered (run Turret Calibration)");
        telemetry.addLine(String.format("leftStop %s pos=%s   (closed %.2f / open %.2f)", leftStop == null ? "MISSING" : "ok",
                Double.isNaN(leftPos) ? "-" : String.format("%.2f", leftPos), LEFT_STOP_CLOSED, LEFT_STOP_OPEN));
        telemetry.addLine(String.format("rightStop %s pos=%s  (closed %.2f / open %.2f)", rightStop == null ? "MISSING" : "ok",
                Double.isNaN(rightPos) ? "-" : String.format("%.2f", rightPos), RIGHT_STOP_CLOSED, RIGHT_STOP_OPEN));
        telemetry.addLine("+ intake/feeder = balls IN; + shooter = launch, RPM positive.");
        telemetry.update();
    }

    private static void set(DcMotorEx m, double p) {
        if (m != null) m.setPower(p);
    }

    /** @param velScale ticks/s -> displayed unit (1 = ticks/s, 60/28 = RPM for the shooter) */
    private static String line(String name, DcMotorEx m, double velScale) {
        if (m == null) return name + ": MISSING from robot config";
        return String.format("%s: pwr=%.2f ticks=%d vel=%.0f%s cur=%.2fA", name, m.getPower(), m.getCurrentPosition(),
                m.getVelocity() * velScale, velScale == 1 ? " t/s" : " rpm", m.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void stop() {
        set(intake, 0); set(feeder, 0); set(shooter, 0);
        turret.stopNow();
    }
}
