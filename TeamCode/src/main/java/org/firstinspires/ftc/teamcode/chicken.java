package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Chicken")
class Chicken extends LinearOpMode {
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

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);

        waitForStart();

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double frontLeftPower = Range.clip((0.5 + 1 + 1) / 3, -1.0, 1.0);
            double frontRightPower = Range.clip((0.5 - 0.5 - 1) / 3, -1.0, 1.0);
            double backLeftPower = Range.clip((0.5 + 1 - 1) / 3, -1.0, 1.0);
            double backRightPower = Range.clip((0.5 - 0.5 + 1) / 3, -1.0, 1.0);

            frontLeft.set(1);
            frontRight.set(1);
            backLeft.set(1);
            backRight.set(1);

            telemetry.addData("Front Left Power", 1);
            telemetry.addData("Front Right Power", 1);
            telemetry.addData("Back Left Power", 1);
            telemetry.addData("Back Right Power", 1);
            telemetry.update();
        }
    }

}
