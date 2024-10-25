package org.firstinspires.ftc.teamcode;
package com.arcrobotics.ftclib.purepursuit;

import org.firstinspires.ftc.teamcode.geometry.Line;
import org.firstinspires.ftc.teamcode.team7786.geometry.Point;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.PointTurnWaypoint;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.WaypointBase;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.waypoints.WaypointTypes;
import org.firstinspires.ftc.teamcode.team7786.utils.GraphFunctions;
import static org.firstinspires.ftc.teamcode.team7786.core.ROBOT_DATA.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class breakTrhough extends LinearOpMode {
    public WaypointBase A;
    public WaypointBase B;
    private WaypointTypes type;

    public void PursuitLine (WaypointBase A, WaypointBase B) {
        super(A.getPoint(), B.getPoint());
        this.A = A;
        this.B = B;
        this.type = B.getType();
        fixIndeterminant(INDETERMINANT_OFFSET);
    }

    public PursuitLine (PointTurnWaypoint A) {
        super(A.getPoint(), A.getPoint());
        this.A = A;
        this.B = A;
    }

    public WaypointTypes getType () { return type; }

    public Point closestPoint(Point P) {
        return GraphFunctions.getClosestPointOnLine(A.getPoint(), B.getPoint(), P);
    }

    public Point[] getCircleIntersections(Point O, double radius) {
        return GraphFunctions.getCircleLineIntersectionPoints(A.getPoint(), B.getPoint(), O, radius);
    }

    public Point[] getOrderedCircleIntersections (Point O, double radius) {
        Point[] rawPoints = getCircleIntersections(O, radius);
        switch (rawPoints.length) {
            case 0: return rawPoints;
            case 1: return rawPoints;
            case 2: return orderDoubleSetByDistance(rawPoints);
        }
        throw new RuntimeException("More than two circle-line intersection points. How is this possible?");
    }

    private Point[] orderDoubleSetByDistance(Point[] raw) {
        if (raw[0].distToPoint(B.getRawPoint()) < raw[1].distToPoint(B.getRawPoint())) {
            return new Point[]{raw[1], raw[0]};
        } else {
            return raw;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}