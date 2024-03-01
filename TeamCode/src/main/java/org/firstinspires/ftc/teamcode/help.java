package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

//the problem child presents itself...


@Autonomous(name = "Mogging")
public class help extends LinearOpMode {

    // Constants
    private static final double TRACKWIDTH = 13.7;
    private static final double TICKS_TO_INCHES = 15.3;
    private static final double CENTER_WHEEL_OFFSET = 2.4;

    @Override
    public void runOpMode() {
        // init hardware and odometry
        initializeHardware();
        initializeOdometry();
        //big brain tbh

        // wait for start
        waitForStart();

        // Autonomous routine
        while (opModeIsActive()) {
            // while match woah

        }
    }
 
    private void initializeHardware() {
        // Member variables, so like motors and stuff
        MotorEx frontLeft = new MotorEx(hardwareMap, "front_left");
        // Initialize other motors and encoders...
    }

    private void initializeOdometry() {
        MotorEx encoderLeft = new MotorEx(hardwareMap, "left_encoder");
        MotorEx encoderRight = new MotorEx(hardwareMap, "right_encoder");
        MotorEx encoderPerp = new MotorEx(hardwareMap, "center_encoder");

        // Set distance per pulse for encoders
        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        // Create the odometry object
        HolonomicOdometry holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        // Create the odometry subsystem
        OdometrySubsystem odometry = new OdometrySubsystem(holOdom);
    }
}