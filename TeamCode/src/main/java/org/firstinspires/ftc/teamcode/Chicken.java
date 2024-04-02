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

            double frontLeftPower = 0;
            double frontRightPower = 0;
            double backLeftPower = 0;
            double backRightPower = 0;


            double strafe = (Math.abs(gamepad1.left_stick_x) >= 0.9) ? Math.signum(gamepad1.left_stick_x) : 0;
            double turn = (Math.abs(gamepad1.right_stick_x) >= 0.9) ? Math.signum(gamepad1.right_stick_x) : 0;


            if (strafe != 0) {
                frontLeftPower = Range.clip(-1 + 1, -1.0, 1.0);
                frontRightPower = Range.clip(1 - 1, -1.0, 1.0);
                backLeftPower = Range.clip(1 + 1, -1.0, 1.0);
                backRightPower = Range.clip(-1 - 1, -1.0, 1.0);
            } else if (turn != 0) {
                frontLeftPower = Range.clip(1, -1.0, 1.0);
                frontRightPower = Range.clip(-1, -1.0, 1.0);
                backLeftPower = Range.clip(1, -1.0, 1.0);
                backRightPower = Range.clip(-1, -1.0, 1.0);
            }


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
