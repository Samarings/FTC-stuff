package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

/**
 * goBILDA PWM RGB lights (DECODE calibration). Optional: missing hardware is ignored.
 *
 * Meaning used by the BIOBUZZ OpModes:
 *  GREEN  = canShoot (aimed, at speed, valid target)
 *  YELLOW = flywheel ready, not aimed / blocked
 *  ORANGE = flywheel off or spinning up
 *  RED    = fault / do not shoot (no target, TIP settling)
 */
public class Leds {
    public enum Color { RED, ORANGE, YELLOW, GREEN }

    private final Servo left, right;
    private Color current = null;

    public Leds(HardwareMap hw) {
        left = tryGet(hw, LED_LEFT_NAME);
        right = tryGet(hw, LED_RIGHT_NAME);
    }

    private static Servo tryGet(HardwareMap hw, String name) {
        try {
            Servo s = hw.get(Servo.class, name);
            if (s instanceof ServoImplEx) {
                ServoImplEx ex = (ServoImplEx) s;
                ex.setPwmRange(new PwmControl.PwmRange((int) LED_PWM_MIN_US, (int) LED_PWM_MAX_US));
                ex.setPwmEnable();
            }
            return s;
        } catch (Exception e) {
            return null;
        }
    }

    public void set(Color c) {
        if (c == current) return;
        current = c;
        double us;
        switch (c) {
            case RED: us = LED_RED_US; break;
            case YELLOW: us = LED_YELLOW_US; break;
            case GREEN: us = LED_GREEN_US; break;
            default: us = LED_ORANGE_US; break;
        }
        double pos = (Range.clip(us, LED_PWM_MIN_US, LED_PWM_MAX_US) - LED_PWM_MIN_US)
                / (LED_PWM_MAX_US - LED_PWM_MIN_US);
        if (left != null) left.setPosition(pos);
        if (right != null) right.setPosition(pos);
    }

    public Color get() { return current; }
}
