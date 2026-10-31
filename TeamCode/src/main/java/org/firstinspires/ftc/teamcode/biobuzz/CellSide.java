package org.firstinspires.ftc.teamcode.biobuzz;

/** The two CELLS of one HIVE: the one nearer the audience wall, and the one nearer the far wall. */
public enum CellSide {
    AUDIENCE, FAR;

    public CellSide other() {
        return this == AUDIENCE ? FAR : AUDIENCE;
    }
}
