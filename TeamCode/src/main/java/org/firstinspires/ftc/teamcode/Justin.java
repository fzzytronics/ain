package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Justin extends LinearOpMode {
    private Motor.Encoder encoderLeft;
    private Motor.Encoder encoderRight;
    private Motor.Encoder encoderCenter;
    private ServoEx m_gyro;

    public static class OdometrySubsystem extends SubsystemBase {
        private Pose2d pose;
        double getX;
        double getY;
        double getHeading;

        public OdometrySubsystem(double initialX, double initialY) {
            this.pose = new Pose2d();
        }

        public Pose2d getPose() {
            return pose;
        }
        public void periodic() {
            //KMS
        }

    }


    OdometrySubsystem odometrySubsystem = new OdometrySubsystem(0, 0);

    private static final double TRACKWIDTH = 13.7;
    private static final double TICKS_TO_INCHES = 15.3;
    private static final double CENTER_WHEEL_OFFSET = 2.4;

    @Override
    public void runOpMode() {
        initializeHardware() ;{
            MotorEx frontLeft = new MotorEx(hardwareMap, "front_left");
            MotorEx frontRight = new MotorEx(hardwareMap, "front_right");
            MotorEx backLeft = new MotorEx(hardwareMap, "back_left");
            MotorEx backRight = new MotorEx (hardwareMap, "back_right");
            MotorEx encoderLeft = new MotorEx(hardwareMap, "encoderLeft");
            MotorEx encoderRight = new MotorEx(hardwareMap, "encoderRight");
            MotorEx encoderCenter = new MotorEx(hardwareMap, "encodeCenter");
        }
        initializeOdometry();
        waitForStart();

        while (opModeIsActive()) {
            Pose2d pose = odometrySubsystem.getPose();
            telemetry.addData("X Position (in)", pose.getX());
            telemetry.addData("Y Position (in)", pose.getY());
            telemetry.addData("Angle", pose.getHeading());
            telemetry.update();
        }
    }

    @NonNull
    private MecanumDriveWheelSpeeds getMecanumDriveWheelSpeeds() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                encoderLeft.getRate(), encoderCenter.getRate(),
                encoderLeft.getRate(), encoderRight.getRate()
        );
        return wheelSpeeds;
    }

    private void initializeHardware() {
        // Member variables, so like motors and stuff
        MotorEx frontLeft = new MotorEx(hardwareMap, "front_left");
        MotorEx frontRight = new MotorEx(hardwareMap, "front_right");
        MotorEx backLeft = new MotorEx(hardwareMap, "back_left");
        MotorEx backRight = new MotorEx(hardwareMap, "back_right");

        MotorEx encoderLeft = new MotorEx(hardwareMap, "encoderLeft");
        MotorEx encoderRight = new MotorEx(hardwareMap, "encoderRight");
        MotorEx encoderCenter = new MotorEx(hardwareMap, "encoderCenter");
        // Initialize other motors and encoders...
    }

    private void initializeOdometry() {
        MotorEx encoderLeft = new MotorEx(hardwareMap, "encoderLeft");
        MotorEx encoderRight = new MotorEx(hardwareMap, "encoderRight");
        MotorEx encoderCenter = new MotorEx(hardwareMap, "encoderCenter");

        // Set distance per pulse for encoders
        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderCenter.setDistancePerPulse(TICKS_TO_INCHES);

        // Create the odometry object
        HolonomicOdometry holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderCenter::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        // Create the odometry subsystem
        OdometrySubsystem odometry = new OdometrySubsystem(0,0);
    }
}