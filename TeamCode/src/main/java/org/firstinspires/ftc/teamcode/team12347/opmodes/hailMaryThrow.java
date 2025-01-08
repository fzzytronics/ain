package org.firstinspires.ftc.teamcode.team12347.opmodes;

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
import org.firstinspires.ftc.teamcode.team12347.geometry.Point;

@Autonomous(name = "hailMaryThrow", group = "Autonomous")
public class hailMaryThrow extends OpMode {

    private AutoBot autoBot;
    private Trajectory currentTraj;

    DcMotorEx dave;
    double velocity;

    @Override
    public void init() {
        HardwareMap hwMap = hardwareMap;
        autoBot = new AutoBot(hwMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Initialization loop logic here
    }

    @Override
    public void start() {
        // Define your waypoints here using WaypointBase objects
        WaypointBase[] waypoints = new WaypointBase[]{
                new StartWaypoint(-60, 25),
                new StandardWaypoint(-30, -10)
                // Add more waypoints as needed
        };

        PursuitLine line1 = new PursuitLine(waypoints[0], waypoints[1]);
        // Add more lines as needed

        PursuitLine[] lines = {line1};

        // Initialize the trajectory with the pursuit lines
        currentTraj = new Trajectory(lines);
        autoBot.beginTrajectory(currentTraj);

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // This should control how the robot follows the trajectory

        autoBot.pursuePose(target);
        /**if (currentTraj != null && currentTraj.hasNextPoint()) {
            PursuitPoint target = currentTraj.nextPoint();
            autoBot.pursuePose(target);
        }
        **/

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
    @Override
    public void stop() {
        // Stop logic here
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
