package org.firstinspires.ftc.teamcode.team7786.purepursuit;


import org.firstinspires.ftc.teamcode.team7786.geometry.Point;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.EndWaypoint;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.PointTurnWaypoint;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.StandardWaypoint;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.WaypointBase;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.WaypointTypes;

import static org.firstinspires.ftc.teamcode.team7786.core.ROBOT_DATA.*;
import static org.firstinspires.ftc.teamcode.team7786.utils.GraphFunctions.*;

import java.util.ArrayList;

public class Trajectory {
    PursuitLine[] paths;

    /** Constructor for trajectory class. Utilized with a builder pattern
     * @param paths        An array of all PursuitLines in this trajectory.
     */
    private Trajectory(PursuitLine[] paths)  {
        this.paths = paths;
        verifyPath();
    }

    /** Verifies that the PursuitLine Array is legal. Checks the following conditions
     * 1. The first path waypoint must be the only start point
     * 2. The last waypoint must be the only end waypoint
     */
    private void verifyPath() {
        for (int i = 0; i < paths.length; i++) {
            if (i == paths.length - 1 && paths[i].getType() != WaypointTypes.END) {
                throw new RuntimeException("Illegal trajectory: No WaypointTypes.End present");
            } else if (i == 0 && paths[0].A.getType() != WaypointTypes.START) {
                throw new RuntimeException("Illegal trajectory: No WaypointTypes.Start present");
            } else if (paths[i].getType() == WaypointTypes.START || paths[i].getType() == WaypointTypes.END) {
                throw new RuntimeException("Illegal trajectory: Start or End Waypoint (WaypointTypes) is present in the middle of trajectory");
            }
        }
    }

    /** Returns a PursuitPoint based on the current stage of the trajectory, updating the stage as needed
     * @param currentPath  The index of the current stage of the pursuit
     * @param pos          The position of the robot
     * @return             The PursuitPoint that the robot should follow
     */
    public PursuitPoint getPursuitPoint(int currentPath, Point pos) {
        switch (paths[currentPath].getType()) {
            case STANDARD: return standardPursuit(currentPath, pos);
            case END: return endPursuit(currentPath, pos);
            case POINT_TURN: return pointTurn(currentPath, pos);
        }
        throw new RuntimeException("Unidentifiable path type. Make sure the switch in getPursuitPoint(Trajectory) is up to date");
    }

    /** Returns a PursuitPoint for a standard pursuit line
     * @param currentPath   The index of the current state
     * @param pos           The current position of the robot
     * @return              The PursuitPoint the robot should follow
     */
    public PursuitPoint standardPursuit(int currentPath, Point pos) {
        Point closestOnLine = paths[currentPath].closestPoint(pos);
        if (Math.abs(closestOnLine.distToPoint(pos)) > MAXIMUM_FOLLOW_RANGE) {  // If outside the acceptable follow range
            return new PursuitPoint(closestOnLine, currentPath);
        }
        Point[] closestOnThisLine = paths[currentPath].getOrderedCircleIntersections(closestOnLine, DEFAULT_FOLLOW_RADIUS);
        Point[] closestOnNextLine = paths[currentPath + 1].getOrderedCircleIntersections(closestOnLine, DEFAULT_FOLLOW_RADIUS);

        if (closestOnNextLine.length > 0) {
            return new PursuitPoint(closestOnNextLine[closestOnNextLine.length - 1], currentPath + 1);
        }
        return new PursuitPoint(closestOnThisLine[closestOnThisLine.length - 1], currentPath);
    }

    /** Returns a PursuitPoint for a end pursuit line
     * @param currentPath   The index of the current state
     * @param pos           The current position of the robot
     * @return              The PursuitPoint the robot should follow
     */
    public PursuitPoint endPursuit(int currentPath, Point pos) {
        if (pos.distToPoint(paths[currentPath].B.getRawPoint()) <= DEFAULT_FOLLOW_RADIUS) {
            return new PursuitPoint(paths[currentPath].B.getPoint(), -1);
        }
        return standardPursuit(currentPath, pos).appendHeading(paths[currentPath].B.getHeading());
    }

    /** Returns a PursuitPoint for a point turn pursuit line
     * @param currentPath   The index of the current state
     * @param pos           The current position of the robot
     * @return              The PursuitPoint the robot should follow
     */
    public PursuitPoint pointTurn(int currentPath, Point pos) {
        double difference = subtractAngles(robotPose.theta, paths[currentPath].B.getHeading());
        if (Math.abs(difference) <= ROTATIONAL_ERROR_MARGIN) {
            return new PursuitPoint(paths[currentPath].B.getPoint(), currentPath + 1);
        }
        return new PursuitPoint(paths[currentPath].B.getPoint(), currentPath, paths[currentPath].B.getHeading());
    }

    /** This class is used to create a builder pattern for the trajectory class */
    public static class Builder {
        private ArrayList<PursuitLine> m_paths;
        private StartWaypoint start;

        /** Start the trajectory build pattern
         * @param startX   the starting x position of this trajectory
         * @param startY   the starting y position of this trajectory
         */
        public Builder(double startX, double startY) {
            m_paths = new ArrayList<PursuitLine>();
            start = new StartWaypoint(startX, startY);
        }

        /** Add a standard waypoint to a trajectory
         * @param x    the x position of this waypoint
         * @param y    the y position of this waypoint
         */
        public Builder toPoint(double x, double y) {
            if (m_paths.size() == 0) {
                m_paths.add(new PursuitLine(start, new StandardWaypoint(x, y)));
                return this;
            }
            m_paths.add(new PursuitLine(getLastWaypoint(), new StandardWaypoint(x, y)));
            return this;
        }

        /** Add a standard waypoint to a trajectory
         * @param heading    the heading that the robot should turn to
         */
        public Builder turnTo(double heading) {
            m_paths.add(new PursuitLine(new PointTurnWaypoint(getLastWaypoint(), heading)));
            return this;
        }

        /** Build the trajectory
         * @param x         The ending x position of this trajectory
         * @param y         The ending y position of this trajectory
         * @param heading   The ending heading of this trajectory
         * @return          The created trajectory
         */
        public Trajectory endAt(double x, double y, double heading) {
            m_paths.add(new PursuitLine(getLastWaypoint(), new EndWaypoint(x, y, heading)));
            return new Trajectory((PursuitLine[]) m_paths.toArray());
        }

        /** Returns the last created waypoint
         * @return   the last created waypoint
         */
        private WaypointBase getLastWaypoint() {
            return m_paths.get(m_paths.size() - 1).B;
        }
    }
}
