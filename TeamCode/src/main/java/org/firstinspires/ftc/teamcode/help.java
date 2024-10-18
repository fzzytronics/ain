//this is now full spaghetti code, best of luck, o programmers
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//the problem child presents itself...

//mew
@Autonomous(name = "Mogging")

public class help extends CommandOpMode {

    // Constants
    private static final double TRACKWIDTH = 13.7;
    private static final double WHEEL_DIAMETER = 0.075;
    private static final double CENTER_WHEEL_OFFSET = 2.4;
    private static final double TICKS_PER_INCH = 15.3;

    // Odometry
    private MecanumDrive driveTrain;
    private HolonomicOdometry odometry;
    private OdometrySubsystem odometrySubsystem;
    private Path mPath;
    private double strafeSpeed;
    private double forwardSpeed;
    private double turn;
    private double heading;
    private PurePursuitCommand ppCommand;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private MotorEx encoderLeft, encoderRight, encoderCenter;
//yurr b
    //rebase test

    @Override
    public void initialize() {
        // Initialization
        initializeHardware();
        initializeOdometry();

        waitForStart();

        while (opModeIsActive()) {
            // Autonomous actions
            // Update odometry
            Pose2d pose = odometry.getPose();
            telemetry.addData("X Position (in)", pose.getX());
            telemetry.addData("Y Position (in)", pose.getY());
            telemetry.addData("Angle", pose.getHeading());
            odometry.updatePose();

            mPath.followPath(driveTrain, odometry);
            telemetry.update();

            ppCommand.schedule(); // Schedule the command
            mPath.init();
        }
    }

    private void initializeHardware() {
        // Initialize motors
        // Hardware components
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");
        frontRight = new MotorEx(hardwareMap, "frontRight");

        Motor intake = new Motor(hardwareMap, "intake");

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
        encoderLeft = new MotorEx(hardwareMap, "frontLeft");
        encoderRight = new MotorEx(hardwareMap, "frontRight");
        encoderCenter = new MotorEx(hardwareMap, "backLeft");

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
    }
}