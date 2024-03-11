package org.firstinspires.ftc.teamcode;
//justin is not life
//hel
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public abstract class Justin extends LinearOpMode{
    public class OdometrySubsystem extends SubsystemBase {
        private MotorEx frontLeft;
        private MotorEx frontRight;
        private MotorEx backLeft;
        private MotorEx backRight;
        private MotorEx encoderLeft;
        private MotorEx encoderRight;
        private MotorEx encoderPerp;
        private HolonomicOdometry holOdom;
     }
        // Constants
        private static final double TRACKWIDTH = 13.7;
        private static final double TICKS_TO_INCHES = 15.3;
        private static final double CENTER_WHEEL_OFFSET = 2.4;

        @Override
        public void runOpMode() {
            // init hardware and odometry
            initializeHardware( );
            initializeOdometry();
            //big brain tbh


            // wait for start
            waitForStart();
            Odometry.resetPose();//?
            // Autonomous routine
            while (opModeIsActive()) {
                // while match woah

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
                OdometrySubsystem odometry = new OdometrySubsystem(hol0dom);
            }
            //update
                telemetry.addData("X Position (in)", Odometry.getX());
                telemetry.addData("Y Position (in)", Odometry.getY());
                telemetry.addData("Theta (deg)", Math.toDegrees(Odometry.getTheta()));
                telemetry.update();
                Odometry.update();
        }
}