package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Chicken")
public class Chicken extends LinearOpMode {
    private Motor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {

        frontLeft = new Motor(hardwareMap, "front_left");
        frontRight = new Motor(hardwareMap, "front_right");
        backLeft = new Motor(hardwareMap, "back_left");
        backRight = new Motor(hardwareMap, "back_right");

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        waitForStart();

        while (opModeIsActive()) {

            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
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

            frontLeft.set(frontLeftPower);
            frontRight.set(frontRightPower);
            backLeft.set(backLeftPower);
            backRight.set(backRightPower);

            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }
    }
}