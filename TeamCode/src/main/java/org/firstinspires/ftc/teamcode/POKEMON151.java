/*
// package org.firstinspires.ftc.teamcode;
//justin is not life
//hel


import static com.arcrobotics.ftclib.geometry.Pose2d.*;

import androidx.annotation.NonNull;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.google.blocks.ftcrobotcontroller.runtime.BNO055IMUAccess;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous
//do not have it be an abstract class
public class POKEMON151 extends LinearOpMode{
    private Motor.Encoder m_backLeftEncoder;
    private Motor.Encoder m_backRightEncoder;
    private Motor.Encoder right_encoder;
    private ServoEx m_gyro;


    public static class OdometrySubsystem extends SubsystemBase {
        public OdometrySubsystem() {
        }

        public class Pose2d {


            public void Pose2d(double x,
                               double y) {
                double getX;
                double getY;
                double getHeading;
            }
        }
        private Pose2d Pose2d() {
            Pose2d Pose2d = new Pose2d();//iwi idk wgaat im doing w my life
            return Pose2d;
        }
        private com.arcrobotics.ftclib.geometry.Pose2d getPoseMeters() {
            getPoseMeters().getX();
            getPoseMeters().getY();
            getPoseMeters().getHeading();
            return null;
        }
        /*private void telemetry;{
            telemetry.addData ("getX"), com.arcrobotics.ftclib.geometry.Pose2d.getX();
            telemetry.addData ("getY"), com.arcrobotics.ftclib.geometry.Pose2d.getY();
            telemetry.addData ("getAngle"), com.arcrobotics.ftclib.geometry.Pose2d.getHeading();
        }*/
        ///i want to die rn
  /*      public class periodic{
            // Get my wheel positionswheelPositions;


            public periodic() {
                // Get the rotation of the robot from the gyro.
                var gyroAngle = m_gyro.getRotation2d();

                // Update the pose
                m_pose = m_odometry.update(gyroAngle,
                        new SwerveModulePosition[] {
                                m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                                m_backLeftModule.getPosition(), m_backRightModule.getPosition()
                        });
            }
            }
        }
        public class MecanumDriveWheels{


        }
        /**
         * Call this at the end of every loop
         */
    /*    public static void update() {
            OdometrySubsystem.update();
        }
    }
    // Constants
    private static final double TRACKWIDTH = 13.7;
    private static final double TICKS_TO_INCHES = 15.3;
    private static final double CENTER_WHEEL_OFFSET = 2.4;


    @Override
    //REMEMBER: runOpMode() --> LinearOpMode
    //initialize() is for other stuff, namely CommandOpMode
    public void runOpMode() {
        // init hardware and odometry
        initializeHardware( );
        initializeOdometry();
        //big brain tbh


         {
            // this snippet from ftclib doubled the errors D:
               /*
               encoders are , m_encoders are also undeclared
               Pose2d is also unrecognized, as well as anything gyro, which is WPlib and FTC core, not ftclib
               You also cant appundeclaredly update to odometry, so yall missing something
               also the code has no idea what periodic means, make sure everything yall need is defined and/or imported
                */




      /*      MecanumDriveWheelSpeeds wheelSpeeds = getMecanumDriveWheelSpeeds();

            // Get my gyro angle.
            Rotation2d gyroAngle = Rotation2d.fromDegrees(m_gyro.getAngle());

            // Update the pose
            Pose2d.update(gyroAngle, wheelSpeeds);
        }


        // wait for start
        waitForStart();
        Pose2d.reset();//?
        // Autonomous routine
        while (opModeIsActive()) {
            /*
             while match woah
            update WHY IS IT RED :(
            /not much red 
            */
/*
            Pose2d getPoseMeters = null;
            telemetry.addData("X Position (in)", getPoseMeters.getX());
            telemetry.addData("Y Position (in)", getPoseMeters.getY());
            Telemetry.Item addData = telemetry.addData("Theta (deg)", Math.toDegrees(getPoseMeters.getHeading()));
            telemetry.update();



        }
    }


    @NonNull
    private MecanumDriveWheelSpeeds getMecanumDriveWheelSpeeds() {
        Motor.Encoder left_encoder = null;
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                (left_encoder.getRate(), right_encoder.getRate(),
                        m_backLeftEncoder.getRate(), m_backRightEncoder.getRate()
                );
        return wheelSpeeds;
    }


    private void initializeHardware() {
        // Member variables, so like motors and stuff
        MotorEx frontLeft = new MotorEx(hardwareMap, "front_left");
        MotorEx frontRight = new MotorEx(hardwareMap, "front_right");
        MotorEx backLeft = new MotorEx(hardwareMap, "back_left");
        MotorEx backRight = new MotorEx (hardwareMap, "back_right");
        MotorEx encoderLeft = new MotorEx(hardwareMap, "left_encoder");
        MotorEx encoderRight = new MotorEx(hardwareMap, "right_encoder");
        MotorEx encoderPerp = new MotorEx(hardwareMap, "center_encoder");
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
        OdometrySubsystem odometry = new OdometrySubsystem(/*=??*//*);
    }
}

OTHER JUSTIN
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//testing testing one two three
@Autonomous
public class Justin extends LinearOpMode {
    Motor motor = new Motor(hardwareMap, "motor encoder");
    Motor.Encoder encoder = motor.encoder;
    Motor.Encoder encoderLeft;
    Motor.Encoder encoderRight;
    Motor.Encoder encoderCenter;
    ServoEx m_gyro;

    public static class OdometrySubsystem extends SubsystemBase {
        Pose2d pose;
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
        initializeHardwareMap();
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
    private void initializeHardwareMap()
    {
        Motor encoder = new Motor(hardwareMap, "encoder");
        Motor frontLeft = new Motor(hardwareMap, "front_left");
        Motor frontRight = new Motor(hardwareMap, "front_right");
        Motor backLeft = new Motor(hardwareMap, "back_left");
        Motor backRight = new Motor(hardwareMap, "back_right");
        Motor encoderLeft = new Motor(hardwareMap, "encoderLeft");
        Motor encoderRight = new Motor(hardwareMap, "encoderRight");
        Motor encoderCenter = new Motor(hardwareMap, "encodeCenter");
    }



    private MecanumDriveWheelSpeeds getMecanumDriveWheelSpeeds() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                encoderLeft.getRate(), encoderCenter.getRate(),
                encoderLeft.getRate(), encoderRight.getRate()
        );
        return wheelSpeeds;
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
        OdometrySubsystem odometry = new OdometrySubsystem(0, 0);
    }
}
*/