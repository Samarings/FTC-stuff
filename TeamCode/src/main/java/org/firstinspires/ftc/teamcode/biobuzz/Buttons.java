package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.HashMap;
import java.util.Map;

/** Rising-edge detection for gamepad buttons: {@code if (b.pressed("g1a", gamepad1.a))}. */
public class Buttons {
    private final Map<String, Boolean> prev = new HashMap<>();
    private final Map<String, Double> heldSince = new HashMap<>();

    public boolean pressed(String key, boolean now) {
        Boolean p = prev.put(key, now);
        return now && (p == null || !p);
    }

    /** True once when the button has been held continuously for {@code seconds}. */
    public boolean heldFor(String key, boolean now, double seconds, double nowSec) {
        if (!now) {
            heldSince.remove(key);
            prev.put(key + "#held", false);
            return false;
        }
        Double since = heldSince.get(key);
        if (since == null) { heldSince.put(key, nowSec); return false; }
        boolean fired = Boolean.TRUE.equals(prev.get(key + "#held"));
        if (!fired && nowSec - since >= seconds) {
            prev.put(key + "#held", true);
            return true;
        }
        return false;
    }
}
