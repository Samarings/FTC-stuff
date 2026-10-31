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
 * Limelight web UI setup (fixed turret): "camera pose in robot space" relative to the robot
 * center on the floor: forward = TURRET_AXIS_OFFSET_FWD + 0.1165 m, side =
 * TURRET_AXIS_OFFSET_LEFT, up = CAMERA_HEIGHT_M, pitch +37°. MegaTag2 then returns the robot
 * center directly (don't subtract the turret offset again).
 */
public class Vision {

    /** What we can see of one CELL's tag cluster this frame. */
    public static class ClusterView {
        public int tagCount = 0;
        /** Average horizontal angle of the visible tags (deg, + = target right of center). */
        public double txDeg = 0.0;
        public double tyDeg = 0.0;
        /** Horizontal distance from the turret axis to the cluster, inches (NaN if unknown). */
        public double distanceIn = Double.NaN;
        public boolean usedPose3d = false;
        int pose3dCount = 0;

        public boolean visible() { return tagCount > 0; }
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
        if (ll == null) { connected = false; return; }
        connected = ll.isConnected();

        // MegaTag2 needs the camera's yaw in the Limelight field frame.
        ll.updateRobotOrientation(Field.pedroHeadingToLimelightYawDeg(robotHeadingDegPedro + turretAngleDeg));

        LLResult r = ll.getLatestResult();
        if (r == null || !r.isValid()) return;
        resultValid = true;

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
                accumulate(cv, f);
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
                fix.stalenessMs = r.getStaleness();
                fix.totalLatencyMs = fix.stalenessMs + r.getCaptureLatency() + r.getTargetingLatency();
                fix.captureTimeSec = nowSec - fix.totalLatencyMs / 1000.0;
                // TURRET_FIXED: the camera pose already includes the turret offset, so this is
                // the robot center. (Rotating turret later: subtract the rotated offset here.)
                poseFix = fix;
            }
        }
    }

    // running sums kept in the view fields, averaged in finish()
    private static void accumulate(ClusterView cv, LLResultTypes.FiducialResult f) {
        cv.tagCount++;
        cv.txDeg += f.getTargetXDegrees();
        cv.tyDeg += f.getTargetYDegrees();
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
        int n = cv.tagCount;
        cv.txDeg /= n;
        cv.tyDeg /= n;
        if (cv.usedPose3d) {
            cv.distanceIn /= cv.pose3dCount;
        } else {
            // Fallback: ty trigonometry with the measured camera height and pitch.
            double dh = TAG_CLUSTER_HEIGHT_IN - CAMERA_HEIGHT_M * Field.M_TO_IN;
            double tan = Math.tan(Math.toRadians(CAMERA_PITCH_DEG + cv.tyDeg));
            cv.distanceIn = tan > 1e-3 ? dh / tan + CAMERA_FORWARD_OF_TURRET_M * Field.M_TO_IN : Double.NaN;
        }
        // NOTE: with fewer than 4 tags visible the average is offset from the true cluster
        // center (tags sit ±2.75 / ±6.5 in along the sticker). Aim tolerance absorbs this;
        // correct it once the tag order on the sticker is confirmed.
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
}
