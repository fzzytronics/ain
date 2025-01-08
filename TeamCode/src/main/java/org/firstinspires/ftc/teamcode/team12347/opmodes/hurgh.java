package org.firstinspires.ftc.teamcode.team12347.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team12347.core.AutoBot;
import org.firstinspires.ftc.teamcode.team12347.geometry.Point;
import org.firstinspires.ftc.teamcode.team12347.purepursuit.PursuitLine;
import org.firstinspires.ftc.teamcode.team12347.purepursuit.PursuitPoint;
import org.firstinspires.ftc.teamcode.team12347.purepursuit.Trajectory;
import org.firstinspires.ftc.teamcode.team12347.purepursuit.waypoints.StandardWaypoint;
import org.firstinspires.ftc.teamcode.team12347.purepursuit.waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.team12347.purepursuit.waypoints.WaypointBase;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;


public class hurgh extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new SimpleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }
}