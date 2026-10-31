package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.geometry.Pose;

/** Our alliance color. Everything is written for RED and converted with rotate180(). */
public enum Alliance {
    RED, BLUE;

    public Alliance opponent() {
        return this == RED ? BLUE : RED;
    }

    /** Convert a red-side pose to this alliance (identity for red, rotate180 for blue). */
    public Pose fromRed(Pose redPose) {
        return this == RED ? redPose : Field.rotate180(redPose);
    }

    /** First AprilTag ID of the given CELL's cluster (cluster = id .. id+3). */
    public int firstTagId(CellSide side) {
        if (this == RED) return side == CellSide.FAR ? 30 : 34;
        return side == CellSide.AUDIENCE ? 38 : 42;
    }

    /** Our up CELL at match start (manual §10.3.1): red = audience, blue = far. */
    public CellSide startUpCell() {
        return this == RED ? CellSide.AUDIENCE : CellSide.FAR;
    }

    /** Which of this alliance's CELLS a tag ID belongs to, or null if it isn't ours. */
    public CellSide cellForTag(int id) {
        for (CellSide s : CellSide.values()) {
            int first = firstTagId(s);
            if (id >= first && id <= first + 3) return s;
        }
        return null;
    }

    /** Which alliance owns a tag ID (30-37 red, 38-45 blue), or null. */
    public static Alliance ownerOfTag(int id) {
        if (id >= 30 && id <= 37) return RED;
        if (id >= 38 && id <= 45) return BLUE;
        return null;
    }
}
