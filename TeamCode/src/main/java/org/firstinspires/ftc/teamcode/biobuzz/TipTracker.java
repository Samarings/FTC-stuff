package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.RobotConstants.*;

/**
 * Knows which of our CELLS is up, counts TIPS, and tells everyone when the HIVES are moving.
 *
 * Only an up-facing CELL's tags face the floor, so the cluster we can see marks the up CELL.
 * If our visible cluster switches CELLS for TIP_CONFIRM_FRAMES frames in a row, a TIP happened.
 * The opponent's cluster is watched the same way, only to black out vision corrections while
 * their HIVE swings.
 *
 * Fallback without tags: our shots into the current CELL are counted, and "tip likely" is
 * shown at BALLS_TO_TIP (our partner shoots into the same CELL, so this is only an estimate).
 */
public class TipTracker {
    private final Alliance alliance;

    private CellSide upCell;
    private int tipCount = 0;
    private double lastOurTipSec = -1e9;
    private double lastHiveMotionSec = -1e9;

    private int ourSwitchFrames = 0;
    private CellSide theirSeen = null;
    private CellSide theirCandidate = null;
    private int theirSwitchFrames = 0;

    private int ourShotsIntoCell = 0;

    public TipTracker(Alliance alliance, CellSide initialUpCell) {
        this.alliance = alliance;
        this.upCell = initialUpCell;
    }

    public void update(Vision vision, double nowSec) {
        // ---- ours
        boolean seeUp = vision.ourCluster(upCell).visible();
        boolean seeOther = vision.ourCluster(upCell.other()).visible();
        if (seeOther && !seeUp) {
            ourSwitchFrames++;
            if (ourSwitchFrames >= TIP_CONFIRM_FRAMES) {
                registerOurTip(nowSec);
            }
        } else {
            ourSwitchFrames = 0;
        }

        // ---- theirs (only to know when their HIVE is moving)
        CellSide seen = null;
        boolean a = vision.theirCluster(CellSide.AUDIENCE).visible();
        boolean f = vision.theirCluster(CellSide.FAR).visible();
        if (a != f) seen = a ? CellSide.AUDIENCE : CellSide.FAR;
        if (seen != null) {
            if (theirSeen == null) {
                theirSeen = seen;
            } else if (seen != theirSeen) {
                if (seen == theirCandidate) theirSwitchFrames++;
                else { theirCandidate = seen; theirSwitchFrames = 1; }
                if (theirSwitchFrames >= TIP_CONFIRM_FRAMES) {
                    theirSeen = seen;
                    theirSwitchFrames = 0;
                    lastHiveMotionSec = nowSec;
                }
            } else {
                theirSwitchFrames = 0;
            }
        }
    }

    private void registerOurTip(double nowSec) {
        upCell = upCell.other();
        tipCount++;
        lastOurTipSec = nowSec;
        lastHiveMotionSec = nowSec;
        ourSwitchFrames = 0;
        ourShotsIntoCell = 0;
    }

    /** Drive team override: flip which CELL is up (field set up differently, or missed TIP). */
    public void manualFlip(double nowSec, boolean countAsTip) {
        if (countAsTip) registerOurTip(nowSec);
        else { upCell = upCell.other(); ourShotsIntoCell = 0; ourSwitchFrames = 0; }
    }

    public void onShotsIntoCell(int n) { ourShotsIntoCell += n; }

    public CellSide getUpCell() { return upCell; }
    public int getTipCount() { return tipCount; }
    public void setTipCount(int n) { tipCount = n; }
    public Alliance getAlliance() { return alliance; }

    /** True while our HIVE is still swinging after a TIP: don't shoot (manual §10.5.1). */
    public boolean inSettleWindow(double nowSec) {
        return nowSec - lastOurTipSec < TIP_SETTLE_S;
    }

    /** True within VISION_TIP_BLACKOUT_S of any HIVE motion (ours or theirs). */
    public boolean hiveRecentlyMoved(double nowSec) {
        return nowSec - lastHiveMotionSec < VISION_TIP_BLACKOUT_S;
    }

    public int shotsIntoCell() { return ourShotsIntoCell; }
    public int ballsToTipEstimate() { return Math.max(0, BALLS_TO_TIP - ourShotsIntoCell); }
    public boolean tipLikely() { return ourShotsIntoCell >= BALLS_TO_TIP; }
}
