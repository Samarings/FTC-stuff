package org.firstinspires.ftc.teamcode.biobuzz.calibration;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.biobuzz.Buttons;

/**
 * Measures the turret's TURRET_TICKS_PER_DEG, direction and cable limits. The motor is set to
 * FLOAT and is NEVER powered: you turn the turret BY HAND.
 *
 * Robot on blocks, ALL cables connected. Run this before setting TURRET_FIXED = false: the
 * turret is never powered until TURRET_TICKS_PER_DEG is set.
 *
 * Steps (gamepad 1):
 *   A   turret on its HOME mark (camera facing robot-forward) -> set home
 *   X   turned COUNTER-CLOCKWISE (seen from above) to the +90° mark (camera facing robot-LEFT)
 *   B   back through home, turned CLOCKWISE to the -90° mark (camera facing robot-RIGHT)
 *       -> ticks/deg from both, averaged. Negative = flip TURRET_REVERSED.
 *   LB  turned CCW until a cable just gets tight -> record the + cable limit
 *   RB  turned CW until a cable just gets tight  -> record the - cable limit
 *       -> suggested soft limits = cable limits minus TURRET_CAL_SAFETY_MARGIN_DEG
 *       The code uses ±225° (team spec): the cables must still be loose at ±225° on the live
 *       angle readout. If they get tight earlier, lower TURRET_MIN_DEG / TURRET_MAX_DEG.
 * Type the results into RobotConstants (TURRET_TICKS_PER_DEG, TURRET_REVERSED, limits), then
 * set TURRET_FIXED = false. Re-check after any wiring change.
 */
@TeleOp(name = "Turret Calibration", group = "Calibration")
public class TurretCalibration extends OpMode {
    private DcMotorEx motor;
    private final Buttons buttons = new Buttons();

    private Integer home = null;
    private Double tpdCcw = null, tpdCw = null;
    private Double limitPlusTicks = null, limitMinusTicks = null;

    @Override
    public void init() {
        try {
            motor = hardwareMap.get(DcMotorEx.class, TURRET_NAME);
            motor.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0.0);
        } catch (Exception e) {
            motor = null;
        }
    }

    @Override
    public void init_loop() { show(); }

    @Override
    public void loop() { show(); }

    private void show() {
        if (motor == null) {
            telemetry.addLine("No motor named '" + TURRET_NAME + "' in the robot configuration.");
            telemetry.addLine("Add it (or change RobotConstants.TURRET_NAME) and re-run.");
            telemetry.update();
            return;
        }
        motor.setPower(0.0); // never powered
        int t = motor.getCurrentPosition();

        if (buttons.pressed("a", gamepad1.a)) home = t;
        if (home != null) {
            if (buttons.pressed("x", gamepad1.x)) tpdCcw = (t - home) / 90.0;
            if (buttons.pressed("b", gamepad1.b)) tpdCw = (home - t) / 90.0;
            if (buttons.pressed("lb", gamepad1.left_bumper)) limitPlusTicks = (double) (t - home);
            if (buttons.pressed("rb", gamepad1.right_bumper)) limitMinusTicks = (double) (t - home);
        }

        telemetry.addLine("TURRET CALIBRATION (hand-turn only, motor never powered)");
        telemetry.addLine("Robot on blocks, ALL cables connected.");
        telemetry.addData("Raw ticks", t);
        telemetry.addData("Home ticks (A)", home == null ? "not set: put turret on mark, press A" : home.toString());

        Double tpd = average(tpdCcw, tpdCw);
        telemetry.addData("ticks/deg CCW (X)", fmt(tpdCcw));
        telemetry.addData("ticks/deg CW (B)", fmt(tpdCw));
        if (tpd != null) {
            telemetry.addData("=> TURRET_TICKS_PER_DEG", String.format("%.3f", Math.abs(tpd)));
            if (tpd < 0) telemetry.addLine("⚠ Angle goes DOWN when turning CCW: set TURRET_REVERSED = "
                    + !TURRET_REVERSED + " (then re-run)");
            if (tpdCcw != null && tpdCw != null && Math.abs(tpdCcw - tpdCw) > 0.05 * Math.abs(tpd))
                telemetry.addLine("⚠ CCW and CW differ by >5%: repeat both (backlash or slipping belt?)");
        }

        double useTpd = tpd != null ? tpd : TURRET_TICKS_PER_DEG;
        if (home != null && useTpd != 0) {
            double ang = (t - home) / useTpd;
            telemetry.addData("Live angle (deg, unwrapped)", String.format("%.1f", ang));
            telemetry.addLine("At the home mark this must read ~0.");
            if (limitPlusTicks != null || limitMinusTicks != null) {
                Double plus = limitPlusTicks == null ? null : limitPlusTicks / useTpd;
                Double minus = limitMinusTicks == null ? null : limitMinusTicks / useTpd;
                telemetry.addData("+ cable limit (LB)", fmt(plus));
                telemetry.addData("- cable limit (RB)", fmt(minus));
                if (plus != null) telemetry.addData("=> TURRET_MAX_DEG", String.format("%.0f", plus - TURRET_CAL_SAFETY_MARGIN_DEG));
                if (minus != null) telemetry.addData("=> TURRET_MIN_DEG", String.format("%.0f", minus + TURRET_CAL_SAFETY_MARGIN_DEG));
                if ((plus != null && plus - TURRET_CAL_SAFETY_MARGIN_DEG < TURRET_MAX_DEG)
                        || (minus != null && minus + TURRET_CAL_SAFETY_MARGIN_DEG > TURRET_MIN_DEG))
                    telemetry.addLine("⚠ Cables get tight inside the current limits (" + TURRET_MIN_DEG + " / "
                            + TURRET_MAX_DEG + "): use the => values above. The 360° flip needs at least ±180°.");
            }
        } else {
            telemetry.addLine("Live angle: set home (A) and measure ticks/deg (X, B) first.");
        }
        telemetry.addLine("Currently in RobotConstants: ticks/deg=" + TURRET_TICKS_PER_DEG
                + "  limits " + TURRET_MIN_DEG + " / " + TURRET_MAX_DEG
                + "  TURRET_FIXED=" + TURRET_FIXED);
        telemetry.addLine("When done: turret back on the home mark (0° = camera facing robot-forward).");
        telemetry.update();
    }

    private static Double average(Double a, Double b) {
        if (a != null && b != null) return 0.5 * (a + b);
        return a != null ? a : b;
    }

    private static String fmt(Double v) {
        return v == null ? "-" : String.format("%.3f", v);
    }

    @Override
    public void stop() {
        if (motor != null) motor.setPower(0.0);
    }
}
