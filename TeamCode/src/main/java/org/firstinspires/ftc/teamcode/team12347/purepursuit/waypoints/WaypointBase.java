package org.firstinspires.ftc.teamcode.team12347.purepursuit.waypoints;

import org.firstinspires.ftc.teamcode.team12347.geometry.Point;

public abstract class WaypointBase {

    /** Returns the type of waypoint that this is
     * @return the waypoint type
     * */
    public abstract WaypointTypes getType();

    /** Returns a point with the same coordinates as this Waypoint
     * @return   a new point with the same coordinates as this waypoint
     */
    public abstract Point getPoint();

    /** Returns a point of this Waypoint
     * @return   the point assigned to this waypoint
     */
    public abstract Point getRawPoint();

    /** Returns the target heading
     * @return   the target heading, or NAN if none exists.
     */
    public abstract double getHeading();

}
