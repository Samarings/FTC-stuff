package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake motor -> feeder motor -> flywheel. No servos: the FEEDER MOTOR is the gate that
 * decides when a ball reaches the flywheel.
 *
 * Modes:
 *  - HOLD:    both motors off.
 *  - COLLECT: intake pulls balls in; feeder stays at FEEDER_COLLECT_POWER (0 by default) so
 *             nothing is pushed into the flywheel.
 *  - FEED:    intake + feeder pulsed to push balls into the flywheel. Only the caller
 *             decides when FEED is allowed (canShoot gate, G417).
 *  - EJECT:   both motors reversed.
 *  - MANUAL:  driver sets intake and feeder power directly (TeleOp triggers).
 *
 * There is no ball sensor, so the ball count is an ESTIMATE: set it when you know
 * (4 pre-loads, after collecting a full source) and it drops by one per detected shot.
 * COLLECT is refused while the estimate is already at MAX_BALLS (G407).
 */
public class Feeder {
    public enum Mode { HOLD, COLLECT, FEED, EJECT, MANUAL }

    private final DcMotor intake, feeder;

    private Mode mode = Mode.HOLD;

    private double manualIntake = 0.0, manualFeeder = 0.0;
    private boolean pulseOn = false;
    private double nextToggleSec = 0.0;

    private int ballEstimate = 0;
    private boolean ballCountKnown = false;

    public Feeder(HardwareMap hw) {
        intake = hw.get(DcMotor.class, INTAKE_NAME);
        feeder = hw.get(DcMotor.class, FEEDER_NAME);

        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        feeder.setDirection(FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // BRAKE on the feeder helps it hold balls back from the flywheel when stopped.
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0.0);
        feeder.setPower(0.0);
    }

    /** Request a mode. COLLECT is downgraded to HOLD when we already hold MAX_BALLS (G407). */
    public void setMode(Mode m) {
        if (m == Mode.COLLECT && ballCountKnown && ballEstimate >= MAX_BALLS) m = Mode.HOLD;
        if (m == Mode.FEED && mode != Mode.FEED) {
            pulseOn = true;
            nextToggleSec = -1.0; // set on next update
        }
        mode = m;
    }

    /** Driver control: run the intake and feeder at these powers (+ = toward the flywheel). */
    public void setManual(double intakePower, double feederPower) {
        mode = Mode.MANUAL;
        manualIntake = intakePower;
        manualFeeder = feederPower;
    }

    public void update(double nowSec) {
        double in = 0.0, fd = 0.0;
        switch (mode) {
            case HOLD:
                break;
            case COLLECT:
                in = INTAKE_COLLECT_POWER;
                fd = FEEDER_COLLECT_POWER;
                break;
            case FEED:
                if (nextToggleSec < 0) nextToggleSec = nowSec + FEED_PULSE_ON_SEC;
                if (nowSec >= nextToggleSec) {
                    pulseOn = !pulseOn;
                    nextToggleSec = nowSec + (pulseOn ? FEED_PULSE_ON_SEC : FEED_PULSE_OFF_SEC);
                }
                if (pulseOn) {
                    in = FEED_INTAKE_POWER;
                    fd = FEED_FEEDER_POWER;
                }
                break;
            case EJECT:
                in = -INTAKE_EJECT_POWER;
                fd = -FEEDER_EJECT_POWER;
                break;
            case MANUAL:
                in = manualIntake;
                fd = manualFeeder;
                break;
        }
        intake.setPower(in);
        feeder.setPower(fd);
    }

    /** Motors off now (end of AUTO / STOP). */
    public void stopNow() {
        mode = Mode.HOLD;
        intake.setPower(0.0);
        feeder.setPower(0.0);
    }

    // ------------------------------------------------------------------ ball estimate

    public void setBallEstimate(int n) {
        ballEstimate = Math.max(0, Math.min(MAX_BALLS, n));
        ballCountKnown = true;
    }

    public void forgetBallCount() {
        ballCountKnown = false;
    }

    /** Call with the number of new shots the Shooter detected this loop. */
    public void onShotsLaunched(int n) {
        if (ballCountKnown) ballEstimate = Math.max(0, ballEstimate - n);
    }

    public int getBallEstimate() { return ballEstimate; }
    public boolean isBallCountKnown() { return ballCountKnown; }
    public Mode getMode() { return mode; }
    /** True while balls are being pushed toward the flywheel (for shot counting). */
    public boolean isFeeding() { return mode == Mode.FEED || (mode == Mode.MANUAL && manualFeeder > 0.05); }
}
