package org.firstinspires.ftc.teamcode.team7786.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team7786.purepursuit.PursuitPoint;
import org.firstinspires.ftc.teamcode.team7786.purepursuit.Trajectory;

import static org.firstinspires.ftc.teamcode.team7786.core.ROBOT_DATA.*;
import static org.firstinspires.ftc.teamcode.team7786.utils.GraphFunctions.*;

public class AutoBot extends MecanumRobot{

    private Trajectory currentTraj;

    public AutoBot (HardwareMap hwmap) {
        super(hwmap);
    }

    public void beginTrajectory(Trajectory traj) {
        currentTraj = traj;
    }

    private void pursuePose(PursuitPoint target) {
        if (Double.isNaN(target.heading)) { // If there is no specified heading
            pursuePoint(target, 1);
            return;
        }

        double minTurnStartDist = GET_ROTATIONAL_START_POINT(Math.abs(subtractAngles(target.heading, robotPose.theta)));
        if (robotPose.distToPoint(target.getPoint()) > minTurnStartDist) {
            pursuePoint(target, 1);
            return;
        }

        driveTowardsPoint(target.x, target.y, target.heading, 1);
    }
}
