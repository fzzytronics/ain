package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class The extends LinearOpMode {
    private static final double LIFTY_POWER = 1.0;

    private boolean dpadUpPressed = false;
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;

    @Override
    public void runOpMode() {
        Gamepad gamepad1 = this.gamepad1;
        Gamepad gamepad2 = this.gamepad2;

        Motor hanger = new Motor(hardwareMap, "hanger");
        Motor claw = new Motor(hardwareMap, "claw");
        Motor lifty = new Motor(hardwareMap, "lifty");
        Motor intake = new Motor(hardwareMap, "intake");
        Servo intake_elevation = hardwareMap.get(Servo.class, "intake_elevation");
        /**Servo grab1 = hardwareMap.get(Servo.class, "grab1");
        Servo grab2 = hardwareMap.get(Servo.class, "grab2");**/
        Servo drone = hardwareMap.get(Servo.class, "drone");
        Motor front_left = new Motor(hardwareMap, "front_left");
        Motor front_right = new Motor(hardwareMap, "front_right");
        Motor back_left = new Motor(hardwareMap, "back_left");
        Motor back_right = new Motor(hardwareMap, "back_right");
        HDrive drive = new HDrive(front_left, front_right, back_left, back_right);

        waitForStart();
        intake.setPositionTolerance(1);
        intake_elevation.setPosition(0);
        drone.setPosition(0.0);

        while (opModeIsActive()) {
            initializeMotorsAndServos(hanger, claw, lifty, intake, intake_elevation
                   /** grab1, grab2**/, drone, front_left, front_right, back_left, back_right);

            hangerControl(hanger);
            liftyControl(lifty);
            drivetrainControl(drive);
            intakeElevationControl(intake_elevation);
            /**
            clawControl(grab1, grab2);
             **/
            droneControl(drone);

            double intakePower = gamepad2.right_trigger > 0.2 ? 1.0 : 0.9;
            intake.set(intakePower);



            stopAllMotorsAndServos(hanger, lifty, claw, intake,
                    front_left, front_right, back_left, back_right);
            stopAllServos(intake_elevation /**grab1, grab2**/, drone);
        }
    }

    private void initializeMotorsAndServos(Motor hanger, Motor claw, Motor lifty, Motor intake,
                                           Servo intakeElevation/** Servo grab1, Servo grab2**/,
                                           Servo drone, Motor frontLeft, Motor frontRight,
                                           Motor backLeft, Motor backRight) {

        hanger.setRunMode(Motor.RunMode.VelocityControl);


       /** claw.setRunMode(Motor.RunMode.VelocityControl);**/


        lifty.setRunMode(Motor.RunMode.VelocityControl);


        intake.setRunMode(Motor.RunMode.RawPower);
        intake.set(0);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontLeft.set(0);



        intakeElevation.setPosition(0.0);
/**
        grab1.setPosition(0.0);
        grab2.setPosition(0.0);
**/
        drone.setPosition(0.0);
    }

    private void stopAllMotorsAndServos(Motor... motors) {
        for (Motor motor : motors) {
            motor.set(0);
        }
    }

    private void stopAllServos(Servo... servos) {
        for (Servo servo : servos) {
            servo.setPosition(0);
        }
    }

    private void hangerControl(Motor hanger) {
        hanger.setRunMode(Motor.RunMode.VelocityControl);


        double hangerPower = 1.0;

        if (gamepad1.dpad_up && !dpadUpPressed) {
            hanger.set(hangerPower);
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
            hanger.set(0);
        } else {
            hanger.set(-hangerPower);
        }
    }
    private void liftyControl(Motor lifty) {
        lifty.setRunMode(Motor.RunMode.VelocityControl);

        double leftStickY = -gamepad2.left_stick_x;

        double targetVelocity = LIFTY_POWER * leftStickY;

        lifty.set(targetVelocity);
    }

    private void drivetrainControl(HDrive drive) {
        double driveX = -gamepad1.left_stick_x;
        double driveY = gamepad1.left_stick_y;

        double turn = driveX;

        double leftPower = driveY + turn;
        double rightPower = driveY - turn;

        leftPower = Math.copySign(Math.min(Math.abs(leftPower), 1.0), leftPower);
        rightPower = Math.copySign(Math.min(Math.abs(rightPower), 1.0), rightPower);

        drive.driveRobotCentric(leftPower, rightPower, driveX);

        if (gamepad1.left_bumper) {
            drive.driveRobotCentric(1, -1, -1);
        } else if (gamepad1.right_bumper) {
            drive.driveRobotCentric(-1, 1, 1);
        }

        sleep(20);
    }

    private void intakeElevationControl(Servo intakeElevation) {
        double position = intakeElevation.getPosition();
        double newPosition = position + gamepad2.left_stick_y * 0.02;

        newPosition = Range.clip(newPosition, 0.0, 1.0);

        if (newPosition < 0.2) {
            newPosition = Range.clip(newPosition, 0.0, 0.2);
        } else if (newPosition < 0.4) {
            newPosition = Range.clip(newPosition, 0.2, 0.4);
        } else if (newPosition < 0.6) {
            newPosition = Range.clip(newPosition, 0.4, 0.6);
        } else if (newPosition < 0.8) {
            newPosition = Range.clip(newPosition, 0.6, 0.8);
        } else {
            newPosition = Range.clip(newPosition, 0.8, 1.0);
        }

        intakeElevation.setPosition(newPosition);
    }
/**
    private void clawControl(Servo grab1, Servo grab2) {
        if (gamepad2.dpad_right && !dpadRightPressed) {
            grab1.setPosition(1.0);
            dpadRightPressed = true;
        } else if (gamepad2.dpad_up && !gamepad2.dpad_right) {
            grab1.setPosition(0.0);
            dpadRightPressed = false;
        }

        if (gamepad2.dpad_left && !dpadLeftPressed) {
            grab2.setPosition(1.0);
            dpadLeftPressed = true;
        } else if (gamepad2.dpad_down && !gamepad2.dpad_left) {
            grab2.setPosition(0.0);
            dpadLeftPressed = false;
        }
    }
**/
    private void droneControl(Servo drone) {
        if (gamepad1.dpad_right && !dpadRightPressed) {
            drone.setPosition(1.0);
            dpadRightPressed = true;
        } else if (!gamepad1.dpad_right) {
            dpadRightPressed = false;
        }
    }

    private void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
