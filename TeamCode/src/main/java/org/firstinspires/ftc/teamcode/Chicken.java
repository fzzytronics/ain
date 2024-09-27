package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Chicken")
public class Chicken extends LinearOpMode {
    private Motor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
       backRight = new MotorEx(hardwareMap, "backRight");
       Motor IntakeElevation = new MotorEx(hardwareMap, "IntakeElevation");

               
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        waitForStart();

        while (opModeIsActive()) {

            double strafe = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            double forward = -gamepad1.left_stick_y;


            double clippedStrafe = Range.clip(strafe, -1.0, 1.0);
            double clippedTurn = Range.clip(turn, -1.0, 1.0);
            double clippedForward = Range.clip(forward, -1.0, 1.0);


            double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

            if (Math.abs(clippedStrafe) >= 1.0) {
                frontLeftPower = -clippedStrafe;
                frontRightPower = clippedStrafe;
                backLeftPower = clippedStrafe;
                backRightPower = -clippedStrafe;
            } else if (Math.abs(clippedTurn) >= 1.0) {
                frontLeftPower = clippedTurn;
                frontRightPower = -clippedTurn;
                backLeftPower = clippedTurn;
                backRightPower = -clippedTurn;
            } else {
                frontLeftPower = clippedForward;
                frontRightPower = clippedForward;
                backLeftPower = clippedForward;
                backRightPower = clippedForward;
            }

            /*backRightPower
             */
            frontLeft.set(1);
            frontRight.set(1);
            backLeft.set(1);
            backRight.set(1);

            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();


        }
    }
}