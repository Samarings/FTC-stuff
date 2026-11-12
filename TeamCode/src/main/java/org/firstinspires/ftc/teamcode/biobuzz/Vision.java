package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Limelight 3A wrapper for BIOBUZZ.
 *
 * Pipeline APRILTAG_PIPELINE must keep ALL 16 IDs (30-45): MegaTag2 localization uses every
 * visible cluster. Our alliance's IDs are picked out here, in code, for aiming and TIP
 * detection.
 *
 * Limelight web UI setup, "camera pose in robot space" (underhand design: lens 0.094 m in
 * front of the turret yaw axis, CAMERA_HEIGHT_M = 0.325 m above the tiles, pitch +20°):
 *  - Rotating turret (TURRET_FIXED = false): relative to the TURRET AXIS on the floor:
 *    forward 0.094 m, side 0, up 0.325 m, pitch +20°. MegaTag2 then returns the turret axis,
 *    and update() shifts it to the robot center with TURRET_AXIS_OFFSET.
 *  - Fixed turret: relative to the ROBOT CENTER on the floor: forward = TURRET_AXIS_OFFSET_FWD
 *    + 0.094 m, side = TURRET_AXIS_OFFSET_LEFT, up 0.325 m, pitch +20°. MegaTag2 returns the
 *    robot center directly.
 *
 * Cluster center (the aim point): the 4 tags of a cluster sit at -6.5, -2.75, +2.75, +6.5 in
 * along the strip, in ID order (or exactly reversed, which gives the same center). With all 4
 * visible the center is their average; with 2 or 3 it is worked out from the known spacing of
 * the visible tags; 1 tag is not enough to aim (the solver falls back to odometry).
 */
public class Vision {

    /** Tag positions along the cluster strip, from its center, inches (manual Fig. 9-15). */
    static final double[] STRIP_IN = {-6.5, -2.75, 2.75, 6.5};

    /** What we can see of one CELL's tag cluster this frame. */
    public static class ClusterView {
        public int tagCount = 0;
        /** Cluster CENTER, horizontal angle from the crosshair (deg, + = right). 1 tag: that tag. */
        public double txDeg = 0.0;
        /** Cluster center, vertical angle from the crosshair (deg, + = up). */
        public double tyDeg = 0.0;
        /** Horizontal distance from the turret axis to the cluster, inches (NaN if unknown). */
        public double distanceIn = Double.NaN;
        public boolean usedPose3d = false;
        int pose3dCount = 0;
        // per tag, indexed by position along the strip (ID - first ID of the cluster)
        final boolean[] seen = new boolean[4];
        final double[] tx = new double[4], ty = new double[4];

        public boolean visible() { return tagCount > 0; }
        /** Enough tags (2+) to know where the cluster center is. */
        public boolean aimable() { return tagCount >= 2; }

        public String centerMethod() {
            return tagCount >= 4 ? "average of 4" : tagCount >= 2 ? "spacing of " + tagCount + " tags"
                    : tagCount == 1 ? "1 tag: NOT used for aim (odometry)" : "no tags (odometry)";
        }
    }

    /** One MegaTag2 robot-pose measurement, already in the Pedro frame. */
    public static class PoseFix {
        public Pose pose;            // robot center, Pedro inches (heading = our input, not a correction)
        public int tagCount;
        public double avgDistM;
        public double stalenessMs;
        public double totalLatencyMs; // staleness + capture + targeting
        public double captureTimeSec; // estimated capture time on our clock
    }

    private final Limelight3A ll;
    private Alliance alliance;

    private boolean connected = false;
    private boolean resultValid = false;
    private final Map<CellSide, ClusterView> ours = new EnumMap<>(CellSide.class);
    private final Map<CellSide, ClusterView> theirs = new EnumMap<>(CellSide.class);
    private PoseFix poseFix = null;
    private String visibleIds = "";
    private double stalenessMs = Double.POSITIVE_INFINITY;
    private double captureTimeSec = 0.0;

    public Vision(HardwareMap hw, Alliance alliance) {
        this.alliance = alliance;
        Limelight3A l = null;
        try {
            l = hw.get(Limelight3A.class, LIMELIGHT_NAME);
            l.setPollRateHz(100);
            l.pipelineSwitch(APRILTAG_PIPELINE);
            l.start();
        } catch (Exception e) {
            l = null;
        }
        ll = l;
        for (CellSide s : CellSide.values()) {
            ours.put(s, new ClusterView());
            theirs.put(s, new ClusterView());
        }
    }

    /**
     * Call once per loop, after the localizer update.
     *
     * @param robotHeadingDegPedro odometry heading (Pedro frame)
     * @param turretAngleDeg       turret angle from home (0 while TURRET_FIXED)
     */
    public void update(double robotHeadingDegPedro, double turretAngleDeg, double nowSec) {
        for (CellSide s : CellSide.values()) {
            ours.put(s, new ClusterView());
            theirs.put(s, new ClusterView());
        }
        poseFix = null;
        resultValid = false;
        visibleIds = "";
        stalenessMs = Double.POSITIVE_INFINITY;
        if (ll == null) { connected = false; return; }
        connected = ll.isConnected();

        // MegaTag2 needs the camera's yaw in the Limelight field frame.
        ll.updateRobotOrientation(Field.pedroHeadingToLimelightYawDeg(robotHeadingDegPedro + turretAngleDeg));

        LLResult r = ll.getLatestResult();
        if (r == null || !r.isValid()) return;
        resultValid = true;
        stalenessMs = r.getStaleness();
        double totalLatencyMs = stalenessMs + r.getCaptureLatency() + r.getTargetingLatency();
        captureTimeSec = nowSec - totalLatencyMs / 1000.0;

        List<LLResultTypes.FiducialResult> fids = r.getFiducialResults();
        StringBuilder ids = new StringBuilder();
        if (fids != null) {
            for (LLResultTypes.FiducialResult f : fids) {
                int id = f.getFiducialId();
                ids.append(id).append(' ');
                Alliance owner = Alliance.ownerOfTag(id);
                if (owner == null) continue;
                CellSide side = owner.cellForTag(id);
                ClusterView cv = (owner == alliance ? ours : theirs).get(side);
                accumulate(cv, f, id - owner.firstTagId(side));
            }
        }
        for (CellSide s : CellSide.values()) {
            finish(ours.get(s));
            finish(theirs.get(s));
        }
        visibleIds = ids.toString().trim();

        Pose3D bp = r.getBotpose_MT2();
        if (bp != null) {
            Position p = bp.getPosition().toUnit(DistanceUnit.METER);
            if (!(p.x == 0.0 && p.y == 0.0)) {
                PoseFix fix = new PoseFix();
                fix.pose = Field.limelightToPedro(p.x, p.y, bp.getOrientation().getYaw(AngleUnit.DEGREES));
                fix.tagCount = r.getBotposeTagCount();
                fix.avgDistM = r.getBotposeAvgDist();
                fix.stalenessMs = stalenessMs;
                fix.totalLatencyMs = totalLatencyMs;
                fix.captureTimeSec = captureTimeSec;
                if (!TURRET_FIXED) {
                    // The Limelight's "robot" is the turret axis here: shift to the robot center.
                    // (TURRET_FIXED: the camera pose in the web UI already includes the offset.)
                    double h = Math.toRadians(robotHeadingDegPedro);
                    double fw = TURRET_AXIS_OFFSET_FWD_IN, lf = TURRET_AXIS_OFFSET_LEFT_IN;
                    fix.pose = new Pose(fix.pose.getX() - (fw * Math.cos(h) - lf * Math.sin(h)),
                            fix.pose.getY() - (fw * Math.sin(h) + lf * Math.cos(h)), fix.pose.getHeading());
                }
                poseFix = fix;
            }
        }
    }

    // per-tag angles + a running distance sum, turned into the cluster center in finish()
    private static void accumulate(ClusterView cv, LLResultTypes.FiducialResult f, int slot) {
        if (cv.seen[slot]) return; // same tag reported twice
        cv.seen[slot] = true;
        cv.tx[slot] = f.getTargetXDegrees();
        cv.ty[slot] = f.getTargetYDegrees();
        cv.tagCount++;
        Pose3D t = f.getTargetPoseCameraSpace();
        double d = Double.NaN;
        if (t != null) {
            Position p = t.getPosition().toUnit(DistanceUnit.METER);
            if (!(p.x == 0.0 && p.y == 0.0 && p.z == 0.0)) {
                // Camera space: +x right, +y down, +z out of the lens. Un-pitch to the floor plane.
                double th = Math.toRadians(CAMERA_PITCH_DEG);
                double fwd = p.z * Math.cos(th) + p.y * Math.sin(th) + CAMERA_FORWARD_OF_TURRET_M;
                d = Math.hypot(fwd, p.x) * Field.M_TO_IN;
            }
        }
        if (!Double.isNaN(d)) {
            cv.distanceIn = Double.isNaN(cv.distanceIn) ? d : cv.distanceIn + d;
            cv.usedPose3d = true;
            cv.pose3dCount++;
        }
    }

    private static void finish(ClusterView cv) {
        if (cv.tagCount == 0) return;
        cv.txDeg = centerOf(cv.tx, cv.seen);
        cv.tyDeg = centerOf(cv.ty, cv.seen);
        if (cv.usedPose3d) {
            cv.distanceIn /= cv.pose3dCount;
        } else {
            // Fallback: ty trigonometry with the measured camera height and pitch.
            double dh = TAG_CLUSTER_HEIGHT_IN - CAMERA_HEIGHT_M * Field.M_TO_IN;
            double tan = Math.tan(Math.toRadians(CAMERA_PITCH_DEG + cv.tyDeg));
            cv.distanceIn = tan > 1e-3 ? dh / tan + CAMERA_FORWARD_OF_TURRET_M * Field.M_TO_IN : Double.NaN;
        }
    }

    /**
     * Value at the cluster center (strip position 0) from the visible tags: a least-squares
     * line through (strip position, value), read at 0. 4 tags: their average. 2 tags: the
     * brief's tx_A + (0 - s_A)(tx_B - tx_A)/(s_B - s_A), which also works when both tags are
     * on the same side of the center. 1 tag: that tag's value.
     *
     * @param v    value per tag, indexed by ID - first ID of the cluster
     * @param seen which of the 4 tags are visible
     */
    static double centerOf(double[] v, boolean[] seen) {
        int n = 0;
        double sMean = 0, vMean = 0;
        for (int i = 0; i < 4; i++) {
            if (!seen[i]) continue;
            n++;
            sMean += STRIP_IN[i];
            vMean += v[i];
        }
        if (n == 0) return Double.NaN;
        sMean /= n;
        vMean /= n;
        if (n == 1) return vMean;
        double sv = 0, ss = 0;
        for (int i = 0; i < 4; i++) {
            if (!seen[i]) continue;
            sv += (STRIP_IN[i] - sMean) * (v[i] - vMean);
            ss += (STRIP_IN[i] - sMean) * (STRIP_IN[i] - sMean);
        }
        return vMean - sv / ss * sMean;
    }

    /** Change which alliance's tags count as "ours" (INIT menu). */
    public void setAlliance(Alliance a) {
        alliance = a;
    }

    public void stop() {
        if (ll != null) ll.stop();
    }

    public boolean isConnected() { return connected; }
    public boolean hasResult() { return resultValid; }
    public ClusterView ourCluster(CellSide side) { return ours.get(side); }
    public ClusterView theirCluster(CellSide side) { return theirs.get(side); }
    /** Latest MegaTag2 fix, or null. Not yet gated: the Localizer decides if it's trustworthy. */
    public PoseFix getPoseFix() { return poseFix; }
    public String getVisibleIds() { return visibleIds; }
    /** How old the latest result was when we read it (ms); infinite if there is none. */
    public double getStalenessMs() { return stalenessMs; }
    /** When the latest frame was captured, on the OpMode clock. */
    public double getCaptureTimeSec() { return captureTimeSec; }
}
