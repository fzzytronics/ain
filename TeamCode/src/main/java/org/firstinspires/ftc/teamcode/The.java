package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "The")
public class The extends LinearOpMode {

    private static final double LIFTY_POWER = 0.5;

    private boolean dpadUpPressed = false;
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadDownPressed = false;

    private Servo intakeElevation;
    private Motor intake;

    private double manualControl = 0.0;
    private boolean isManualControlActive = false;
    private Motor front_right;
    private Motor front_left;
    private Motor back_right;
    private Motor back_left;

    @Override
    public void runOpMode() {

        Gamepad gamepad1 = this.gamepad1;
        Gamepad gamepad2 = this.gamepad2;

        Motor hanger = new Motor(hardwareMap, "hanger");
        Motor claw = new Motor(hardwareMap, "claw");
        Motor lifty = new Motor(hardwareMap, "lifty");
        intake = new Motor(hardwareMap, "intake");
        intakeElevation = hardwareMap.get(Servo.class, "intake_elevation");
        Servo drone = hardwareMap.get(Servo.class, "drone");
        Motor front_left = new Motor(hardwareMap, "front_left");
        Motor front_right = new Motor(hardwareMap, "front_right");
        Motor back_left = new Motor(hardwareMap, "back_left");
        Motor back_right = new Motor(hardwareMap, "back_right");
        HDrive drive = new HDrive(front_left, front_right, back_left, back_right);

        initializeMotorsAndServos(hanger, claw, lifty, intake, intakeElevation, drone, front_left, front_right, back_left, back_right);

        waitForStart();

        while (opModeIsActive()) {
            hangerControl(hanger);
            liftyControl(lifty);
            drivetrainControl(drive);
            intakeElevationControl(intakeElevation);
            intakeControl(intake);
            droneControl(drone);

            stopAllMotorsAndServos(hanger, lifty, claw, intake, front_left, front_right, back_left, back_right);
            stopAllServos(intakeElevation, drone);
        }
    }

    private void initializeMotorsAndServos(Motor hanger, Motor claw, Motor lifty, Motor intake, Servo intakeElevation, Servo drone, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        hanger.setRunMode(Motor.RunMode.VelocityControl);
        claw.setRunMode(Motor.RunMode.VelocityControl);
        lifty.setRunMode(Motor.RunMode.VelocityControl);
        intake.setRunMode(Motor.RunMode.RawPower);
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.set(1);
        frontRight.set(1);
        backLeft.set(1);
        backRight.set(1);

        intakeElevation.setPosition(0.0);
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
        double hangerPower = 1.0;

        if (gamepad1.dpad_up && !dpadUpPressed) {
            hanger.set(hangerPower);
            dpadUpPressed = true;
        } else if (gamepad1.dpad_down && !dpadDownPressed) {
            hanger.set(-hangerPower);
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            dpadUpPressed = false;
            dpadDownPressed = false;
            hanger.set(0);
        }
    }

    private void liftyControl(Motor lifty) {
        double leftStickY = -gamepad2.left_stick_x;
        double targetVelocity = LIFTY_POWER * leftStickY;
        lifty.set(targetVelocity);
    }

    private void joystickControl(HDrive drive) {
        double maxPower = 1.0;
        double forwardPower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.left_stick_x;

        DifferentialDrive m_drive = new DifferentialDrive(front_right, front_left);

        double leftSpeed = Range.clip(forwardPower - turnPower, -maxPower, maxPower);
        double rightSpeed = Range.clip(forwardPower + turnPower, -maxPower, maxPower);
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    private void bumperControl(HDrive drive) {
        DifferentialDrive m_drive = new DifferentialDrive(front_right, front_left, back_right, back_left);
        double maxPower = 1.0;

        if (gamepad2.left_bumper) {
            m_drive.tankDrive(-maxPower, maxPower);
        } else if (gamepad2.right_bumper) {
            m_drive.tankDrive(maxPower, -maxPower);
        }
    }

    private void drivetrainControl(HDrive drive) {
        joystickControl(drive);
        bumperControl(drive);
    }


    private void intakeElevationControl(Servo intakeElevation) {
        double elevationPower = 0.0;

        if (gamepad2.dpad_up && !dpadUpPressed) {
            elevationPower = 0.824;
            dpadUpPressed = true;
        } else if (gamepad2.dpad_down && !dpadDownPressed) {
            elevationPower = -0.824;
            dpadDownPressed = true;
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            dpadUpPressed = false;
            dpadDownPressed = false;
        }

        double newPosition = intakeElevation.getPosition() + elevationPower;
        newPosition = Range.clip(newPosition, 0.38, 4.5);
        intakeElevation.setPosition(newPosition);
    }

    private void intakeControl(Motor intake) {
        double intakePower = 0.0;

        if (gamepad1.right_trigger > 0.1) {
            intakePower = 1.0;
        }

        intake.set(intakePower);
    }

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
