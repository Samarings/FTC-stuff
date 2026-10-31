package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The ONE owner of the robot pose. Wraps Pedro's follower (Pinpoint localizer, configured in
 * pedroPathing/Constants.java). Nothing else reads the Pinpoint or sets the pose.
 *
 * Odometry is the main source of position and heading. Limelight MegaTag2 fixes only nudge
 * x/y, every few seconds, after gating, latency compensation, a median window and a partial
 * apply. Heading is never corrected by vision.
 */
public class Localizer {
    private final Follower follower;

    // odometry history for latency compensation: {timeSec, x, y}
    private final ArrayDeque<double[]> history = new ArrayDeque<>();
    // accepted vision offsets: {timeSec, dx, dy}
    private final ArrayList<double[]> window = new ArrayList<>();
    // consecutive "big" windows waiting for agreement: {dx, dy}
    private final ArrayList<double[]> bigWindows = new ArrayList<>();

    private double lastApplySec = -1e9;
    private int correctionsApplied = 0;
    private double lastAppliedDx = 0, lastAppliedDy = 0;
    private Pose lastAcceptedVisionPose = null;
    private double lastMedianOffsetIn = 0.0;
    private final Map<String, Integer> rejects = new LinkedHashMap<>();
    private String lastReject = "";

    public Localizer(HardwareMap hw) {
        follower = Constants.createFollower(hw);
    }

    public Follower follower() { return follower; }

    /**
     * AUTO INIT only, robot still: reset the Pinpoint position + IMU (blocks ~0.3 s inside
     * Pedro), then set the start pose. Legal under G304.H (nothing moves).
     */
    public void resetImuAndSetPose(Pose start) {
        try {
            follower.getPoseTracker().getLocalizer().resetIMU();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        setPose(start);
    }

    /** Set the pose without resetting the IMU (start-preset changes, TeleOp from PoseStorage, re-seed). */
    public void setPose(Pose p) {
        follower.setStartingPose(p);
        follower.setPose(p);
        history.clear();
        window.clear();
        bigWindows.clear();
    }

    /** Call exactly once per loop, before anything reads the pose. */
    public void update(double nowSec) {
        follower.update();
        Pose p = follower.getPose();
        history.addLast(new double[]{nowSec, p.getX(), p.getY()});
        while (!history.isEmpty() && nowSec - history.peekFirst()[0] > ODOM_HISTORY_S) history.removeFirst();
    }

    /**
     * INIT only: read odometry without running the drive controller (nothing may move
     * after INIT, G304.H).
     */
    public void updatePoseOnly(double nowSec) {
        follower.updatePose();
        history.clear();
    }

    public Pose getPose() { return follower.getPose(); }
    public double getHeadingDeg() { return Math.toDegrees(follower.getPose().getHeading()); }
    /** Translation speed, in/s. */
    public double getSpeed() { return follower.getVelocity().getMagnitude(); }
    public double getVelX() { return follower.getVelocity().getXComponent(); }
    public double getVelY() { return follower.getVelocity().getYComponent(); }
    /** Turn rate, deg/s (absolute). */
    public double getTurnRateDeg() { return Math.abs(Math.toDegrees(follower.getAngularVelocity())); }

    // =================================================================== VISION CORRECTIONS

    /**
     * Offer this loop's MegaTag2 fix.
     *
     * @param allowApply false in AUTO while a Pedro path is running (setPose mid-path jerks
     *                   the follower); frames are still collected.
     */
    public void offerVision(Vision.PoseFix fix, TipTracker tips, boolean allowApply, double nowSec) {
        if (!VISION_CORRECTIONS_ENABLED) return;
        String why = gate(fix, tips, nowSec);
        if (why != null) {
            reject(why);
        } else {
            double[] odom = odomAt(fix.captureTimeSec);
            if (odom == null) {
                reject("NO_HISTORY");
            } else {
                lastAcceptedVisionPose = fix.pose;
                window.add(new double[]{nowSec, fix.pose.getX() - odom[0], fix.pose.getY() - odom[1]});
                while (window.size() > VISION_WINDOW_MAX_FRAMES) window.remove(0);
            }
        }
        Iterator<double[]> it = window.iterator();
        while (it.hasNext()) if (nowSec - it.next()[0] > VISION_WINDOW_MAX_AGE_S) it.remove();

        if (allowApply && nowSec - lastApplySec >= VISION_CORRECTION_PERIOD_S
                && window.size() >= VISION_WINDOW_MIN_FRAMES) {
            decide(nowSec);
        }
    }

    private String gate(Vision.PoseFix fix, TipTracker tips, double nowSec) {
        if (fix == null || fix.pose == null) return "NO_FIX";
        if (fix.tagCount < VISION_MIN_TAGS) return "FEW_TAGS";
        if (fix.avgDistM > VISION_MAX_AVG_DIST_M) return "TOO_FAR";
        if (fix.stalenessMs > VISION_MAX_STALENESS_MS) return "STALE";
        if (getSpeed() > VISION_MAX_SPEED_IN_S) return "MOVING";
        if (getTurnRateDeg() > VISION_MAX_TURN_RATE_DEG_S) return "TURNING";
        // TURRET_FIXED: no turret turn-rate gate.
        if (tips != null && tips.hiveRecentlyMoved(nowSec)) return "HIVE_MOVING";
        double x = fix.pose.getX(), y = fix.pose.getY();
        if (!Field.insideField(x, y)) return "OUTSIDE_FIELD";
        if (Field.insideHiveFrame(x, y, 0)) return "IN_HIVE_FRAME";
        return null;
    }

    private void decide(double nowSec) {
        int n = window.size();
        double[] xs = new double[n], ys = new double[n];
        for (int i = 0; i < n; i++) { xs[i] = window.get(i)[1]; ys[i] = window.get(i)[2]; }
        double dx = median(xs), dy = median(ys);
        double mag = Math.hypot(dx, dy);
        lastMedianOffsetIn = mag;
        window.clear();
        lastApplySec = nowSec;

        if (mag < VISION_MIN_OFFSET_IN) {
            bigWindows.clear();
            return;
        }
        if (mag <= VISION_PARTIAL_MAX_IN) {
            bigWindows.clear();
            shiftPose(VISION_ALPHA * dx, VISION_ALPHA * dy);
            return;
        }
        // Big offset: could be a bad map, wrong cell or reflection. Only trust it if several
        // consecutive windows agree (a real error after a collision).
        bigWindows.add(new double[]{dx, dy});
        while (bigWindows.size() > VISION_BIG_AGREE_WINDOWS) bigWindows.remove(0);
        if (bigWindows.size() == VISION_BIG_AGREE_WINDOWS) {
            double mx = 0, my = 0;
            for (double[] b : bigWindows) { mx += b[0]; my += b[1]; }
            mx /= bigWindows.size(); my /= bigWindows.size();
            boolean agree = true;
            for (double[] b : bigWindows) if (Math.hypot(b[0] - mx, b[1] - my) > VISION_BIG_AGREE_IN) agree = false;
            if (agree) {
                shiftPose(mx, my);
                bigWindows.clear();
            }
        }
    }

    private void shiftPose(double dx, double dy) {
        Pose p = follower.getPose();
        follower.setPose(new Pose(p.getX() + dx, p.getY() + dy, p.getHeading()));
        // Shift history too, so frames captured before this correction aren't applied twice.
        for (double[] h : history) { h[1] += dx; h[2] += dy; }
        correctionsApplied++;
        lastAppliedDx = dx;
        lastAppliedDy = dy;
    }

    /** Odometry (x, y) at a past time, linearly interpolated, or null if outside the history. */
    private double[] odomAt(double t) {
        double[] prev = null;
        for (double[] h : history) {
            if (h[0] >= t) {
                if (prev == null) return Math.abs(h[0] - t) < 0.03 ? new double[]{h[1], h[2]} : null;
                double u = (t - prev[0]) / Math.max(1e-6, h[0] - prev[0]);
                return new double[]{prev[1] + u * (h[1] - prev[1]), prev[2] + u * (h[2] - prev[2])};
            }
            prev = h;
        }
        // capture time is after the newest sample (tiny latency): use the newest
        return prev != null && t - prev[0] < 0.05 ? new double[]{prev[1], prev[2]} : null;
    }

    private static double median(double[] v) {
        double[] c = v.clone();
        Arrays.sort(c);
        int n = c.length;
        return n % 2 == 1 ? c[n / 2] : 0.5 * (c[n / 2 - 1] + c[n / 2]);
    }

    private void reject(String why) {
        lastReject = why;
        Integer c = rejects.get(why);
        rejects.put(why, c == null ? 1 : c + 1);
    }

    // =================================================================== TELEMETRY

    public Pose getLastAcceptedVisionPose() { return lastAcceptedVisionPose; }
    public int getCorrectionsApplied() { return correctionsApplied; }
    public double getSecondsSinceCorrection(double nowSec) { return nowSec - lastApplySec; }
    public double getLastMedianOffsetIn() { return lastMedianOffsetIn; }
    public String getLastAppliedText() { return String.format("%.2f, %.2f", lastAppliedDx, lastAppliedDy); }
    public String getLastReject() { return lastReject; }
    public int getWindowSize() { return window.size(); }

    public String rejectSummary() {
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<String, Integer> e : rejects.entrySet()) {
            if (e.getKey().equals("NO_FIX")) continue;
            sb.append(e.getKey()).append('=').append(e.getValue()).append(' ');
        }
        return sb.toString().trim();
    }

    public List<String> describe(double nowSec) {
        List<String> l = new ArrayList<>();
        Pose p = getPose();
        l.add(String.format("Pose  x=%.1f y=%.1f h=%.1f°", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        l.add(String.format("Speed %.1f in/s  turn %.0f°/s", getSpeed(), getTurnRateDeg()));
        Pose v = lastAcceptedVisionPose;
        l.add(v == null ? "Vision pose: none accepted yet"
                : String.format("Vision pose x=%.1f y=%.1f", v.getX(), v.getY()));
        l.add(String.format("Corrections %d (last %s in, %.1fs ago) window=%d median=%.2f",
                correctionsApplied, getLastAppliedText(), getSecondsSinceCorrection(nowSec),
                window.size(), lastMedianOffsetIn));
        l.add("Rejects: " + rejectSummary());
        return l;
    }
}
