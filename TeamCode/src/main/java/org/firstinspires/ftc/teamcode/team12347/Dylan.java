//qiwi's miserable attempt at being a competent programmer
//pathing code
package org.firstinspires.ftc.teamcode.team12347;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import org.firstinspires.ftc.teamcode.Justin2;

@Autonomous(name = "Dylan")
public class Dylan extends CommandOpMode {

    private static final double TRACKWIDTH = 13.7;
    private static final double WHEEL_DIAMETER = 0.075;
    private static final double CENTER_WHEEL_OFFSET = 2.4;
    private static final double TICKS_PER_REV = 15.3;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private HolonomicOdometry odometry;


}