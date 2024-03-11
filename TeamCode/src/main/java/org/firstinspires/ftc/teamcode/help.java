//this is now full spaghetti code, best of luck, o programmers
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

//the problem child presents itself...

//mew
@Autonomous(name = "Mogging")
public abstract class help extends CommandOpMode {
    // Hardware components
    private MotorEx frontLeft, backLeft, backRight, frontRight;
    private Motor intake;

    // Constants
    private static final double TRACKWIDTH = 9.1;
    private static final double WHEEL_DIAMETER = 4.0;
    private static final double CENTER_WHEEL_OFFSET = 2.4;
    private static final double TICKS_PER_INCH = 15.3;

    // Odometry
    private MecanumDrive driveTrain;
    private HolonomicOdometry odometry;
    private OdometrySubsystem odometrySubsystem;
    private Path m_path;
    private double strafeSpeed;
    private double forwardSpeed;
    private double turn;
    private double heading;

    @Override
    public void runOpMode() {

        initializeHardware();
        initializeOdometry();
         waitForStart();

        while (opModeIsActive()) {
            // Autonomous actions
            // Update odometry
            odometry.updatePose();
            m_path.followPath(driveTrain, odometry);
        }

    }

    private void initializeHardware() {
        // Initialize motors
        frontLeft = new MotorEx(hardwareMap, "front_left");
        backLeft = new MotorEx(hardwareMap, "back_left");
        backRight = new MotorEx(hardwareMap, "back_right");
        frontRight = new MotorEx(hardwareMap, "front_right");
        intake = new Motor(hardwareMap, "intake");

        // Initialize drive train
        driveTrain = new MecanumDrive(frontLeft, backLeft, backRight, frontRight);

        // Set motor modes and other configurations

        // Motor locations for kinematics
        Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

        // Creating kinematics object
        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        driveTrain.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);
    }

    private void initializeOdometry() {
        // Initialize encoders
        MotorEx encoderLeft = new MotorEx(hardwareMap, "front_left");
        MotorEx encoderRight = new MotorEx(hardwareMap, "front_right");
        MotorEx encoderCenter = new MotorEx(hardwareMap, "back_left");

        // Set distance per pulse for encoders
        double ticksToInches = WHEEL_DIAMETER * Math.PI / TICKS_PER_INCH;
        encoderLeft.setDistancePerPulse(ticksToInches);
        encoderRight.setDistancePerPulse(ticksToInches);
        encoderCenter.setDistancePerPulse(ticksToInches);

        // Create odometry object
        HolonomicOdometry holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderCenter::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        // Create odometry subsystem
        OdometrySubsystem odometry = new OdometrySubsystem(holOdom);

        // Initial odometry update
        odometry.update();

        // Create pure pursuit command
        PurePursuitCommand ppCommand = new PurePursuitCommand(
                driveTrain, odometry,
                new StartWaypoint(0, 0),
                new GeneralWaypoint(200, 0, 0.8, 0.8, 30),
                new EndWaypoint(400, 0, 0, 0.5, 0.5, 30, 0.8, 1)
        );

        // Schedule the command
        ppCommand.schedule();
    }
}