package org.firstinspires.ftc.teamcode;
//justin is not life
//hel
import statstatic org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
//do not have it be an abstract class
public class Justin extends LinearOpMode{
    public static class OdometrySubsystem extends SubsystemBase {
        public class Pose2d{
            public Pose2d getInstance() {
                Pose2d Pose2d = Pose2d;//iwi idk wgaat im doing w my life
                return Pose2d;
            }

        }

        public class periodic{
            // Get my wheel positions
            var wheelPositions = new MecanumDriveWheelPositions(
                    m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
                    m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance());
        }
        public class MecanumDriveWheels{

        }
        /**
         * Call this at the end of every loop
         */
        public static void update() {
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
            initializePose2d();
            //big brain tbh
            public void periodic(); {
                // this snippet from ftclib doubled the errors D:
                /*
                encoders are undeclared, m_encoders are also undeclared
                Pose2d is also unrecognized, as well as anything gyro, which is WPlib and FTC core, not ftclib
                You also cant apply update to odometry, so yall missing something
                also the code has no idea what periodic means, make sure everything yall need is defined and/or imported
                 */



                MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                        (left_encoder.getRate(), right_encoder.getRate(),
                                m_backLeftEncoder.getRate(), m_backRightEncoder.getRate()
                        );

                // Get my gyro angle.
                Rotation2d gyroAngle = Rotation2d.fromDegrees(m_gyro.getAngle());

                // Update the pose
                Pose2d = Odometry.update(gyroAngle, wheelSpeeds);
            }

            // wait for start
            waitForStart();
            Odometry.reset();//?
            // Autonomous routine
            while (opModeIsActive()) {
                // while match woah
                //update WHY IS IT RED :(
                telemetry.addData("X Position (in)", Odometry.getX());
                telemetry.addData("Y Position (in)", Odometry.getY());
                telemetry.addData("Theta (deg)", Math.toDegrees(Odometry.getTheta()));
                telemetry.update();
                Odometry.update();

                            }
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
                OdometrySubsystem odometry = new OdometrySubsystem(/*=??*/);
            }

        }