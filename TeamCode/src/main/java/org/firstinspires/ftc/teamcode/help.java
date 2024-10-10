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
import com.arcrobotics.ftclib.purepursuit.Waypoint;
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
    private Path m_path;
    private double strafeSpeed;
    private double forwardSpeed;
    private double turn;
    private double heading;
    private PurePursuitCommand ppCommand;
    private MotorEx front_left, front_right, back_left, back_right;
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

            m_path.followPath(driveTrain, odometry);
            telemetry.update();

            ppCommand.schedule(); // Schedule the command
            m_path.init();
        }
    }

    private void initializeHardware() {
        // Initialize motors
        // Hardware components
        front_left = new MotorEx(hardwareMap, "front_left");
        back_left = new MotorEx(hardwareMap, "back_left");
        back_right = new MotorEx(hardwareMap, "back_right");
        front_right = new MotorEx(hardwareMap, "front_right");

        Motor intake = new Motor(hardwareMap, "intake");

        // Initialize drive train
        driveTrain = new MecanumDrive(front_left, back_left, back_right, front_right);

        // Set motor modes and other configurations

        // Motor locations for kinematics
        Translation2d front_leftLocation = new Translation2d(0.381, 0.381);
        Translation2d front_rightLocation = new Translation2d(0.381, -0.381);
        Translation2d back_leftLocation = new Translation2d(-0.381, 0.381);
        Translation2d back_rightLocation = new Translation2d(-0.381, -0.381);

        // Creating kinematics object
        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                front_leftLocation, front_rightLocation, back_leftLocation, back_rightLocation);

        driveTrain.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);
    }

    private void initializeOdometry() {
        // Initialize encoders
        encoderLeft = new MotorEx(hardwareMap, "front_left");
        encoderRight = new MotorEx(hardwareMap, "front_right");
        encoderCenter = new MotorEx(hardwareMap, "back_left");

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