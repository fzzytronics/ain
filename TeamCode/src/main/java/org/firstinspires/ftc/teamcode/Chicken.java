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

            double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

            if (Math.abs(strafe) >= 1.0) {

                frontLeftPower = Range.clip(-strafe, -1.0, 1.0);
                frontRightPower = Range.clip(strafe, -1.0, 1.0);
                backLeftPower = Range.clip(strafe, -1.0, 1.0);
                backRightPower = Range.clip(-strafe, -1.0, 1.0);
            } else if (Math.abs(turn) >= 1.0) {

                frontLeftPower = Range.clip(turn, -1.0, 1.0);
                frontRightPower = Range.clip(-turn, -1.0, 1.0);
                backLeftPower = Range.clip(turn, -1.0, 1.0);
                backRightPower = Range.clip(-turn, -1.0, 1.0);
            } else {

                frontLeftPower = Range.clip(forward, -1.0, 1.0);
                frontRightPower = Range.clip(forward, -1.0, 1.0);
                backLeftPower = Range.clip(forward, -1.0, 1.0);
                backRightPower = Range.clip(forward, -1.0, 1.0);
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
