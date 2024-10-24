package org.firstinspires.ftc.teamcode.team7786.purepursuit;

import org.firstinspires.ftc.teamcode.team7786.geometry.Point;
import org.firstinspires.ftc.teamcode.team7786.geometry.Pose;

public class PursuitPoint extends Point {
    public int state;
    public double heading;

    public PursuitPoint(double x, double y, int state, double heading) {
        super(x, y);
        this.state = state;
        this.heading = heading;
    }

    public PursuitPoint (Point point, int state, double heading) {
        super(point.x, point.y);
        this.state = state;
        this.heading = heading;
    }

    public PursuitPoint (Point point, int state) {
        super(point.x, point.y);
        this.state = state;
    }

    public Point getPoint() {
        return this;
    }

    public PursuitPoint appendHeading(double heading) {
        this.heading = heading;
        return this;
    }
}
