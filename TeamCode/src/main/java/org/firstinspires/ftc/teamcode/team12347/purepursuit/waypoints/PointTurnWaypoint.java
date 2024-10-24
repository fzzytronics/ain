package org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints;

import org.firstinspires.ftc.teamcode.team7786.geometry.Point;

public class PointTurnWaypoint extends WaypointBase {
    private final WaypointTypes type = WaypointTypes.POINT_TURN;
    private Point location;
    private double heading;

    public PointTurnWaypoint (double x, double y, double heading) {
        location = new Point(x, y);
    }

    public PointTurnWaypoint (WaypointBase xy, double heading) {
        location = new Point(xy.getRawPoint().x, xy.getRawPoint().y);
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
