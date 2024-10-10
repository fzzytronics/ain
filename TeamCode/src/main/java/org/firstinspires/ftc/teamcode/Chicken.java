package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Chicken")
public class Chicken extends LinearOpMode {
    private Motor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        MotorEx front_left = new MotorEx(hardwareMap, "front_left");
        MotorEx front_right = new MotorEx(hardwareMap, "front_right");
        MotorEx back_left = new MotorEx(hardwareMap, "back_left");
        MotorEx back_right = new MotorEx(hardwareMap, "back_right");
       Motor IntakeElevation = new MotorEx(hardwareMap, "IntakeElevation");

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        waitForStart();

        while (opModeIsActive()) {

            double strafe = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_y;
            double forward = -gamepad1.left_stick_x;

            double clippedStrafe = Range.clip(strafe, -1.0, 1.0);
            double clippedTurn = Range.clip(turn, -1.0, 1.0);
            double clippedForward = Range.clip(forward, -1.0, 1.0);

            double front_leftPower, front_rightPower, back_leftPower, back_rightPower;

            if (Math.abs(clippedStrafe) >= 1.0) {
                front_leftPower = -clippedStrafe;
                front_rightPower = clippedStrafe;
                back_leftPower = clippedStrafe;
                back_rightPower = -clippedStrafe;
            } else if (Math.abs(clippedTurn) >= 1.0) {
                front_leftPower = clippedTurn;
                front_rightPower = -clippedTurn;
                back_leftPower = clippedTurn;
                back_rightPower = -clippedTurn;
            } else { 
                front_leftPower = clippedForward;
                front_rightPower = clippedForward;
                back_leftPower = clippedForward;
                back_rightPower = clippedForward;
            }
 
            /*backRightPower
             */
            frontLeft.set(1);
            frontRight.set(1);
            backLeft.set(1);
            backRight.set(1);

            telemetry.addData("Front Left Power", front_leftPower);
            telemetry.addData("Front Right Power", front_rightPower);
            telemetry.addData("Back Left Power", back_leftPower);
            telemetry.addData("Back Right Power", back_rightPower);
            telemetry.update();


        }
    }
}