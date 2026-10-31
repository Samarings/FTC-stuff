package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.biobuzz.Buttons;
import org.firstinspires.ftc.teamcode.biobuzz.Turret;

/**
 * Check the intake, feeder, shooter and turret motors one at a time: names,
 * wiring, directions, encoders and current. Directions come from RobotConstants, so what you
 * see here is what the match code does.
 *
 * Gamepad 1:
 *   dpad up/down   select device
 *   left stick Y   power for the selected motor (up = +)
 *   B              stop everything
 *
 * Expected: + intake/feeder power pulls balls IN toward the shooter (else flip
 * INTAKE_REVERSED / FEEDER_REVERSED); + shooter power launches forward and the RPM reads
 * positive (else flip SHOOTER_REVERSED). Shooter power is capped at RPM_CAP / 6000.
 * The turret motor is NEVER powered while TURRET_FIXED; only its encoder is shown.
 */
@TeleOp(name = "Motor Test (intake/feeder/shooter/turret)", group = "Calibration")
public class MotorTest extends OpMode {
    private static final String[] NAMES = {"intake", "feeder", "shooter", "turret"};

    private DcMotorEx intake, feeder, shooter, turretMotor;
    private Turret turret;
    private int sel = 0;
    private final Buttons buttons = new Buttons();
    private double lastSec = -1;

    @Override
    public void init() {
        intake = motor(INTAKE_NAME, INTAKE_REVERSED, DcMotor.ZeroPowerBehavior.BRAKE);
        feeder = motor(FEEDER_NAME, FEEDER_REVERSED, DcMotor.ZeroPowerBehavior.BRAKE);
        shooter = motor(SHOOTER_NAME, SHOOTER_REVERSED, DcMotor.ZeroPowerBehavior.FLOAT);
        turret = new Turret(hardwareMap, null); // handles direction, FLOAT when locked, limits
        try { turretMotor = hardwareMap.get(DcMotorEx.class, TURRET_NAME); } catch (Exception e) { turretMotor = null; }
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

        telemetry.addLine("Selected (dpad): >> " + NAMES[sel] + " <<   stick=" + String.format("%.2f", stick));
        telemetry.addLine(line("intake", intake, 1));
        telemetry.addLine(line("feeder", feeder, 1));
        telemetry.addLine(line("shooter", shooter, 60.0 / SHOOTER_TICKS_PER_REV) + String.format("  (cap pwr %.2f)", shooterCap));
        telemetry.addLine(turret.describe() + (turretMotor != null ? String.format("  %.2fA", turretMotor.getCurrent(CurrentUnit.AMPS)) : ""));
        if (TURRET_FIXED) telemetry.addLine("  turret LOCKED (TURRET_FIXED): never powered, encoder only");
        else if (!Turret.isCalibrated()) telemetry.addLine("  turret not calibrated: never powered (run Turret Calibration)");
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
