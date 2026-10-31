package org.firstinspires.ftc.teamcode.biobuzz;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

/** Panels field view: odometry pose, last accepted vision pose, and the target CELL. */
public final class FieldDraw {
    private FieldDraw() {}

    private static final double R = 9.0;
    private static FieldManager field;
    private static final Style ODOM = new Style("", "#3F51B5", 0.75);
    private static final Style VISION = new Style("", "#FF9800", 0.75);
    private static final Style TARGET = new Style("", "#4CAF50", 0.75);

    public static void init() {
        try {
            field = PanelsField.INSTANCE.getField();
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        } catch (Exception e) {
            field = null;
        }
    }

    public static void draw(Pose odom, Pose vision, Pose target) {
        if (field == null) return;
        robot(odom, ODOM);
        if (vision != null) robot(vision.withHeading(odom.getHeading()), VISION);
        if (target != null) {
            field.setStyle(TARGET);
            field.moveCursor(target.getX(), target.getY());
            field.circle(4);
            field.moveCursor(odom.getX(), odom.getY());
            field.line(target.getX(), target.getY());
        }
        field.update();
    }

    private static void robot(Pose p, Style s) {
        if (p == null || Double.isNaN(p.getX()) || Double.isNaN(p.getY())) return;
        field.setStyle(s);
        field.moveCursor(p.getX(), p.getY());
        field.circle(R);
        Vector v = p.getHeadingAsUnitVector();
        field.moveCursor(p.getX(), p.getY());
        field.line(p.getX() + v.getXComponent() * R, p.getY() + v.getYComponent() * R);
    }
}
