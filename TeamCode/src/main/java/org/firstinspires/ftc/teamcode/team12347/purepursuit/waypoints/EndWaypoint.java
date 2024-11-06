package org.firstinspires.ftc.teamcode.team12347.purepursuit.waypoints;

import org.firstinspires.ftc.teamcode.team12347.geometry.Point;

public class EndWaypoint extends WaypointBase {
    private final WaypointTypes type = WaypointTypes.END;
    private Point location;
    private double heading;

    public EndWaypoint (double x, double y, double heading) {
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
        return heading;
    }
}
