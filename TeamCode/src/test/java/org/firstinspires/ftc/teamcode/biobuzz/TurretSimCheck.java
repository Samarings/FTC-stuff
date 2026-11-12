package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.reflect.Proxy;
import java.util.ArrayDeque;

/**
 * Off-robot check of the turret AprilTag lock: no hardware, no JUnit, just main().
 *
 * It runs the real Turret, Vision.centerOf and ShotSolver.aimDirectionDeg against a simulated
 * turret motor, a simulated Limelight (4 tags, 20° pitch, limited field of view, 45 ms late)
 * and a robot that drives and spins, and fails loudly if the turret leaves ±225°, stops
 * pointing at the cluster, or never flips.
 *
 * Run from the project root (Git Bash) after a build:  bash TeamCode/src/test/run-turret-sim.sh
 *
 * The motor model (speed, lag) is a guess, so this proves the LOGIC (signs, limits, flip,
 * latency, tag-count fallbacks), not the real gains: tune those with "Turret PIDF Tuner".
 */
public class TurretSimCheck {
    static final double DT = 0.015;           // control loop period, s
    static final double TPD = 537.7 * 3 / 360; // sim only: pretend a 3:1 turret
    static final double CAM_FWD_IN = 0.094 * Field.M_TO_IN;
    static final double CAM_H_IN = 0.325 * Field.M_TO_IN;

    /** First-order motor: speed follows power x free speed with a short lag; small powers don't move it. */
    static class FakeMotor {
        double deg = 0, degPerSec = 0, power = 0;
        final DcMotorEx proxy = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (p, m, a) -> {
                    switch (m.getName()) {
                        case "getCurrentPosition": return (int) Math.round(deg * TPD);
                        case "setPower": power = (Double) a[0]; return null;
                        case "getPower": return power;
                        default:
                            Class<?> r = m.getReturnType();
                            return r == double.class ? (Object) 0.0 : r == int.class ? (Object) 0 : r == boolean.class ? (Object) false : null;
                    }
                });

        void step() {
            double freeDegS = TURRET_MOTOR_FREE_TICKS_S / TPD;
            double target = Math.abs(power) < 0.03 ? 0 : power * freeDegS * 0.85; // loaded: slower than free speed
            degPerSec += (target - degPerSec) * DT / 0.08;
            deg += degPerSec * DT;
        }
    }

    public static void main(String[] args) {
        TURRET_FIXED = false;
        TURRET_TICKS_PER_DEG = TPD;

        clusterCenter();
        setpointChoice();
        stepAndFlip();
        System.out.println("-- lock, no field pose (TeleOp): target found from the tags");
        lock(null, 1.0);
        System.out.println("-- lock, field pose with the CELL 5 in off in the map (AUTO): tags correct it");
        lock(new Pose(59.3 + 4, 56.6 - 3, 0), 1.0);
        System.out.println("-- lock without the counter-rotation feedforward (KV = 0): still locks, lags more");
        lock(null, 0.0);
        System.out.println("ALL TURRET SIM CHECKS PASSED");
    }

    // ------------------------------------------------------------------ cluster center

    static void clusterCenter() {
        double[] tx = new double[4];
        for (int i = 0; i < 4; i++) tx[i] = 3.0 + 0.8 * Vision.STRIP_IN[i]; // center at 3.0°
        for (int mask = 1; mask < 16; mask++) {
            boolean[] seen = new boolean[4];
            for (int i = 0; i < 4; i++) seen[i] = (mask >> i & 1) == 1;
            double c = Vision.centerOf(tx, seen);
            if (Integer.bitCount(mask) >= 2) near(c, 3.0, 1e-9, "center from tags mask " + mask);
            else near(c, tx[Integer.numberOfTrailingZeros(mask)], 1e-9, "1 tag returns that tag");
        }
        // the brief's 2-tag formula, both tags on one side
        near(Vision.centerOf(new double[]{0, 0, 5.2, 8.2}, new boolean[]{false, false, true, true}),
                5.2 + (0 - 2.75) * (8.2 - 5.2) / (6.5 - 2.75), 1e-9, "2-tag extrapolation");
        System.out.println("ok  cluster center: 4 tags = average, 2-3 tags = spacing, also one-sided");
    }

    // ------------------------------------------------------------------ setpoint / flip rule

    static void setpointChoice() {
        near(Turret.pickSetpoint(0, 170), 170, 0, "nearest");
        near(Turret.pickSetpoint(170, -175), 185, 0, "keeps turning the same way past 180");
        near(Turret.pickSetpoint(224, -133), 227, 0, "inside the 3° hysteresis: no flip yet (clamped to 225 later)");
        near(Turret.pickSetpoint(224, -131), -131, 0, "past 225+3: flip (+229 -> -131)");
        near(Turret.pickSetpoint(-224, 131), 131, 0, "past -225-3: flip (-229 -> +131)");
        near(Turret.pickSetpoint(-131, -133), -133, 0, "after a flip it stays on the new turn");
        for (double ref = TURRET_MIN_DEG; ref <= TURRET_MAX_DEG; ref += 2.5) {
            for (double yaw = -180; yaw <= 180; yaw += 1.5) {
                double sp = Turret.pickSetpoint(ref, yaw);
                near(Field.wrapDeg(sp - yaw), 0, 1e-9, "same direction");
                check(sp >= TURRET_MIN_DEG - TURRET_FLIP_HYSTERESIS_DEG - 1e-9 && sp <= TURRET_MAX_DEG + TURRET_FLIP_HYSTERESIS_DEG + 1e-9,
                        "setpoint " + sp + " outside the limits (ref " + ref + ", yaw " + yaw + ")");
            }
        }
        System.out.println("ok  setpoint choice: same direction, inside ±225°, flips with hysteresis");
    }

    // ------------------------------------------------------------------ step response + flip

    static void stepAndFlip() {
        FakeMotor m = new FakeMotor();
        Turret t = new Turret(m.proxy, null);
        double now = 0;
        now = drive(t, m, now, 90, 2.0);
        near(m.deg, 90, 2.0, "step 0 -> 90 settles"); // 1.5° = the fake friction band with KF = 0
        check(!t.isFlipping(), "a plain step is not a flip");
        now = drive(t, m, now, 120, 2.0);
        boolean sawFlip = false;
        double min = m.deg, max = m.deg;
        for (int i = 0; i < 4.0 / DT; i++) { // +120 -> -120 is 240 -> past 225: must go round through 0
            t.chooseSetpoint(-120);
            now += DT;
            t.update(now);
            m.step();
            sawFlip |= t.isFlipping();
            min = Math.min(min, m.deg);
            max = Math.max(max, m.deg);
        }
        near(m.deg, -120, 2.0, "flip ends at -120");
        check(sawFlip && !t.isFlipping(), "flip flagged while turning, cleared at the end");
        check(max <= 125 && min >= -125, "flip went through 0, not past +225 (" + min + ".." + max + ")");
        check(t.getFault() == null, "no fault: " + t.getFault());

        // manual: full stick into the limit stops at the limit and never flips
        for (int i = 0; i < 6.0 / DT; i++) {
            t.setManualPower(-1);
            now += DT;
            t.update(now);
            m.step();
        }
        check(m.deg >= TURRET_MIN_DEG - 1.0 && m.deg <= TURRET_MIN_DEG + 3.0, "manual stops at the -225 limit: " + m.deg);
        check(t.getFault() == null, "no fault at the limit: " + t.getFault());
        System.out.println("ok  steps settle, +120 -> -120 flips through 0, manual stops at -225°");
    }

    static double drive(Turret t, FakeMotor m, double now, double yaw, double seconds) {
        for (int i = 0; i < seconds / DT; i++) {
            t.chooseSetpoint(yaw);
            now += DT;
            t.update(now);
            m.step();
        }
        return now;
    }

    // ------------------------------------------------------------------ full lock

    /** One camera frame: per-tag angles as the Limelight would report them. */
    static class Frame {
        double captureSec, readySec, distIn;
        final double[] tx = new double[4], ty = new double[4];
        final boolean[] seen = new boolean[4];
        int count;
    }

    /**
     * Robot drives a circle and spins 3 turns one way then back, tags blocked for 1.5 s and
     * half the cluster hidden for 2 s. The turret must stay on the cluster and inside ±225°.
     *
     * @param mapTarget target position the solver is told (null = no field pose)
     */
    static void lock(Pose mapTarget, double kv) {
        TURRET_KV = kv;
        final double tgtX = 59.3, tgtY = 56.6;
        FakeMotor m = new FakeMotor();
        Turret turret = new Turret(m.proxy, null);
        ShotSolver solver = new ShotSolver(Alliance.RED);
        ArrayDeque<Frame> inFlight = new ArrayDeque<>();
        Frame latest = null;

        double heading = Math.toDegrees(Math.atan2(tgtY - 30, tgtX - 12)) + 10; // tags start in view
        double nextFrameSec = 0, lastFlipSec = -9, worst = 0, worstBlocked = 0, worstNearLimit = 0, minDeg = 0, maxDeg = 0;
        int flips = 0;
        boolean wasFlipping = false;

        for (double now = 0; now < 22; now += DT) {
            double w = now > 2 && now < 14 ? 90 : now > 15 && now < 19 ? -120 : 0; // robot turn rate, deg/s
            heading += w * DT;
            double x = 0 + 12 * Math.cos(0.4 * now), y = 30 + 12 * Math.sin(0.4 * now);
            boolean blocked = now > 10 && now < 11.5;
            boolean halfHidden = now > 5 && now < 7;

            // camera: a frame every 33 ms, delivered 45 ms later
            if (now >= nextFrameSec) {
                nextFrameSec += 0.033;
                Frame f = capture(x, y, heading + m.deg, tgtX, tgtY, halfHidden);
                f.captureSec = now;
                f.readySec = now + 0.045;
                if (!blocked) inFlight.addLast(f);
            }
            while (!inFlight.isEmpty() && inFlight.peekFirst().readySec <= now) latest = inFlight.removeFirst();

            // ---- what Robot.sense() + TurretAimer do each loop
            turret.read(now);
            turret.setRobotYawRate(w);
            boolean tagsOk = latest != null && latest.count >= 2 && (now - latest.readySec) * 1000 <= VISION_MAX_STALENESS_MS;
            double tx = latest == null ? 0 : Vision.centerOf(latest.tx, latest.seen);
            double ty = latest == null ? 0 : Vision.centerOf(latest.ty, latest.seen);
            double bearing = solver.aimDirectionDeg(now, x, y, Field.wrapDeg(heading) + turret.getAimAngleDeg(), mapTarget,
                    tagsOk, tx, ty, latest == null ? Double.NaN : latest.distIn, latest == null ? 0 : latest.captureSec);
            if (Double.isNaN(bearing)) turret.hold();
            else turret.chooseSetpoint(Field.wrapDeg(bearing - heading));
            turret.update(now);
            m.step();

            // ---- score
            minDeg = Math.min(minDeg, m.deg);
            maxDeg = Math.max(maxDeg, m.deg);
            if (turret.isFlipping() && !wasFlipping) flips++;
            if (turret.isFlipping()) lastFlipSec = now;
            wasFlipping = turret.isFlipping();
            double err = Math.abs(Field.wrapDeg(Math.toDegrees(Math.atan2(tgtY - y, tgtX - x)) - heading - m.deg));
            // The last degrees before a limit are slowed on purpose (slow zone), then it flips:
            // score those separately. canShoot is false there anyway (aim error / FLIPPING).
            boolean nearLimit = Math.abs(turret.getSetpointDeg()) > TURRET_MAX_DEG - TURRET_SLOW_ZONE_DEG - 5;
            if (now > 1.0 && now - lastFlipSec > 1.0) {
                if (nearLimit) worstNearLimit = Math.max(worstNearLimit, err);
                else worst = Math.max(worst, err);
                if (blocked && !nearLimit) worstBlocked = Math.max(worstBlocked, err);
            }
            check(turret.getFault() == null, "turret fault at t=" + now + ": " + turret.getFault());
            if (System.getenv("SIM_TRACE") != null && Math.round(now / DT) % 10 == 0)
                System.out.printf("t=%5.2f w=%4.0f turret=%7.1f set=%7.1f err=%5.2f tags=%s n=%d flip=%s pwr=%.2f%n", now, w, m.deg,
                        turret.getSetpointDeg(), err, tagsOk, latest == null ? 0 : latest.count, turret.isFlipping(), turret.getPower());
        }
        System.out.printf("    turret travel %.1f° .. %.1f°, %d flips, worst pointing error %.2f° (%.2f° with the tags blocked, %.2f° in the slow zone before a flip)%n",
                minDeg, maxDeg, flips, worst, worstBlocked, worstNearLimit);
        check(minDeg >= TURRET_MIN_DEG - 2 && maxDeg <= TURRET_MAX_DEG + 2, "turret left ±225°");
        check(flips >= 2, "robot spun 3 turns: expected at least 2 flips, got " + flips);
        // The shot tolerance at this range (~65 in) is atan(6/65) = 5.3°. Without the feedforward
        // a P-only loop lags by (turn rate / loop gain), about 11° at 120°/s with this motor model.
        check(worst < (kv > 0 ? 5.0 : 13.0), "lost the lock: " + worst + "°");
        System.out.println("ok");
    }

    static Frame capture(double axisX, double axisY, double shooterDirDeg, double tgtX, double tgtY, boolean halfHidden) {
        Frame f = new Frame();
        double dir = Math.toRadians(shooterDirDeg), pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double camX = axisX + CAM_FWD_IN * Math.cos(dir), camY = axisY + CAM_FWD_IN * Math.sin(dir);
        double up = TAG_CLUSTER_HEIGHT_IN - CAM_H_IN, distSum = 0;
        for (int i = 0; i < 4; i++) {
            double vx = tgtX - camX, vy = tgtY + Vision.STRIP_IN[i] - camY; // strip along field y
            double fwd = vx * Math.cos(dir) + vy * Math.sin(dir), left = -vx * Math.sin(dir) + vy * Math.cos(dir);
            double z = fwd * Math.cos(pitch) + up * Math.sin(pitch), yUp = -fwd * Math.sin(pitch) + up * Math.cos(pitch);
            f.tx[i] = Math.toDegrees(Math.atan2(-left, z));
            f.ty[i] = Math.toDegrees(Math.atan2(yUp, z));
            f.seen[i] = z > 0 && Math.abs(f.tx[i]) < 27.25 && Math.abs(f.ty[i]) < 20.5 && !(halfHidden && i >= 2);
            if (f.seen[i]) {
                f.count++;
                distSum += Math.hypot(fwd + CAM_FWD_IN, left);
            }
        }
        f.distIn = f.count > 0 ? distSum / f.count * 1.08 : Double.NaN; // Limelight distance reads 8% long
        return f;
    }

    static void near(double got, double want, double tol, String what) {
        check(Math.abs(got - want) <= tol, what + ": got " + got + ", want " + want);
    }

    static void check(boolean ok, String what) {
        if (!ok) throw new AssertionError(what);
    }
}
