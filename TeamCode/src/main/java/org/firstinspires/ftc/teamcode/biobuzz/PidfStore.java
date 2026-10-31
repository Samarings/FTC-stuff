package org.firstinspires.ftc.teamcode.biobuzz;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.lang.reflect.Field;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Saves tuned PIDF gains on the Control Hub and loads them into RobotConstants at startup.
 *
 * File: /sdcard/FIRST/biobuzz_pidf.txt, one "NAME=value" per line.
 *
 * ⚠ While the file exists it OVERRIDES the values typed in RobotConstants. When tuning is
 * done, copy the final numbers into RobotConstants and delete the file (PIDF Tuner: hold
 * BACK for 1 s), otherwise later edits to RobotConstants will seem to "do nothing".
 */
public final class PidfStore {
    private PidfStore() {}

    public static final String FILE = "/sdcard/FIRST/biobuzz_pidf.txt";

    /** The gains that can be tuned and saved, in display order. */
    public static final String[] KEYS = {
            "SHOOTER_KP", "SHOOTER_KI", "SHOOTER_KD", "SHOOTER_KF",
            "HOLD_KP", "HOLD_KI", "HOLD_KD", "HOLD_KF",
    };

    private static Map<String, Double> codeDefaults = null;
    private static boolean loaded = false;
    private static String status = "not loaded";

    /** Load the saved file once per app run (called from Robot). Safe if the file is missing. */
    public static synchronized void loadOnce() {
        rememberDefaults();
        if (loaded) return;
        loaded = true;
        File f = new File(FILE);
        if (!f.exists()) {
            status = "using RobotConstants values (no saved file)";
            return;
        }
        int n = 0;
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            String line;
            while ((line = r.readLine()) != null) {
                int eq = line.indexOf('=');
                if (eq <= 0) continue;
                String k = line.substring(0, eq).trim();
                double v = Double.parseDouble(line.substring(eq + 1).trim());
                if (set(k, v)) n++;
            }
            status = "loaded " + n + " saved gains from " + FILE + " (overrides RobotConstants)";
        } catch (Exception e) {
            status = "could not read " + FILE + ": " + e.getMessage();
        }
    }

    /** Write the current gains to the file. */
    public static synchronized String save() {
        try {
            File f = new File(FILE);
            File dir = f.getParentFile();
            if (dir != null && !dir.exists()) dir.mkdirs();
            try (FileWriter w = new FileWriter(f, false)) {
                for (String k : KEYS) w.write(String.format(Locale.US, "%s=%s%n", k, Double.toString(get(k))));
            }
            loaded = true;
            status = "saved " + KEYS.length + " gains to " + FILE;
        } catch (Exception e) {
            status = "save FAILED: " + e.getMessage();
        }
        return status;
    }

    /** Delete the file and put the RobotConstants (code) values back. */
    public static synchronized String revertToCode() {
        rememberDefaults();
        new File(FILE).delete();
        for (Map.Entry<String, Double> e : codeDefaults.entrySet()) set(e.getKey(), e.getValue());
        status = "deleted saved file; back to RobotConstants values";
        return status;
    }

    public static String status() { return status; }

    public static double get(String key) {
        try {
            return field(key).getDouble(null);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    public static boolean set(String key, double value) {
        rememberDefaults(); // capture compiled-in values before the first change
        try {
            field(key).setDouble(null, value);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public static double codeDefault(String key) {
        rememberDefaults();
        Double d = codeDefaults.get(key);
        return d == null ? Double.NaN : d;
    }

    private static Field field(String key) throws NoSuchFieldException {
        return RobotConstants.class.getField(key);
    }

    /** Capture the compiled-in values before any file overrides them. */
    private static void rememberDefaults() {
        if (codeDefaults != null) return;
        codeDefaults = new LinkedHashMap<>();
        for (String k : KEYS) codeDefaults.put(k, get(k));
    }
}
