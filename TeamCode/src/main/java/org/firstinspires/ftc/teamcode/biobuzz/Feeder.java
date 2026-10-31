package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Intake motor -> feeder motor -> stopper servos -> flywheel (DECODE layout).
 *
 * Modes:
 *  - HOLD:    stoppers closed, motors off.
 *  - COLLECT: stoppers closed, intake + feeder pull balls in.
 *  - FEED:    stoppers open, intake + feeder pulsed to push balls into the flywheel.
 *             Only the caller decides when FEED is allowed (canShoot gate, G417).
 *  - EJECT:   stoppers closed, everything reversed.
 *
 * There is no ball sensor, so the ball count is an ESTIMATE: set it when you know
 * (4 pre-loads, after collecting a full source) and it drops by one per detected shot.
 * COLLECT is refused while the estimate is already at MAX_BALLS (G407).
 */
public class Feeder {
    public enum Mode { HOLD, COLLECT, FEED, EJECT }

    private final DcMotor intake, feeder;
    private final Servo leftStop, rightStop;

    private Mode mode = Mode.HOLD;
    private Boolean stoppersClosed = null;

    private boolean pulseOn = false;
    private double nextToggleSec = 0.0;

    private int ballEstimate = 0;
    private boolean ballCountKnown = false;

    /**
     * @param commandServosNow false in TeleOp INIT so nothing moves during the AUTO->TELEOP
     *                         transition (G403); servos are commanded on the first update().
     */
    public Feeder(HardwareMap hw, boolean commandServosNow) {
        intake = hw.get(DcMotor.class, INTAKE_NAME);
        feeder = hw.get(DcMotor.class, FEEDER_NAME);
        leftStop = hw.get(Servo.class, LEFT_STOP_NAME);
        rightStop = hw.get(Servo.class, RIGHT_STOP_NAME);

        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        feeder.setDirection(FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0.0);
        feeder.setPower(0.0);

        if (commandServosNow) setStoppersClosed(true);
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

    public void update(double nowSec) {
        double in = 0.0, fd = 0.0;
        switch (mode) {
            case HOLD:
                setStoppersClosed(true);
                break;
            case COLLECT:
                setStoppersClosed(true);
                in = INTAKE_COLLECT_POWER;
                fd = FEEDER_COLLECT_POWER;
                break;
            case FEED:
                setStoppersClosed(false);
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
                setStoppersClosed(true);
                in = -INTAKE_EJECT_POWER;
                fd = -FEEDER_EJECT_POWER;
                break;
        }
        intake.setPower(in);
        feeder.setPower(fd);
    }

    /** Motors off now (end of AUTO). Servos keep their position. */
    public void stopNow() {
        mode = Mode.HOLD;
        intake.setPower(0.0);
        feeder.setPower(0.0);
    }

    private void setStoppersClosed(boolean closed) {
        if (stoppersClosed != null && stoppersClosed == closed) return;
        stoppersClosed = closed;
        leftStop.setPosition(closed ? LEFT_STOP_CLOSED : LEFT_STOP_OPEN);
        rightStop.setPosition(closed ? RIGHT_STOP_CLOSED : RIGHT_STOP_OPEN);
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
    public boolean isFeeding() { return mode == Mode.FEED; }
}
