package org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints;

import org.firstinspires.ftc.teamcode.team7786.geometry.Point;

public class StandardWaypoint extends WaypointBase {
    private final WaypointTypes type = WaypointTypes.STANDARD;
    private Point location;

    public StandardWaypoint (double x, double y) {
        location = new Point(x, y);
    }

    public WaypointTypes getType () {
        return type;
    }

    public Point getPoint() {
        return new Point(location.x, location.y);
    }

    public Point getRawPoint() {
        return location;
    }

    public double getHeading () {
        return Double.NaN;
    }
}
