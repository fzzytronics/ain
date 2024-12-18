//this is now full spaghetti code, best of luck, o programmers
/*
package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
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
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import org.firstinspires.ftc.teamcode.Justin2;


//the problem child presents itself...


//mew
@Autonomous(name = "Mogging")


public class help extends CommandOpMode {


    // Constants
    private static final double TRACKWIDTH = 13.7;
    private static final double WHEEL_DIAMETER = 0.075;
    private static final double CENTER_WHEEL_OFFSET = 2.4;
    private static final double TICKS_PER_REV = 15.3;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;


    // Odometry
    private MecanumDrive driveTrain;
    private HolonomicOdometry odometry;
    private Path mPath;
    private double strafeSpeed;
    private double forwardSpeed;
    private double turn;
    private double heading;
    private PurePursuitCommand ppCommand;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private MotorEx encoderLeft, encoderRight, encoderCenter;
    private ServoEx gyro;
    public Translation2d fLeft, fRight, bLeft, bRight;
    public MecanumDriveKinematics kinematics;
    public Pose2d robotPose;
    private Motor intake, lifty;
    private Encoder odoLeft, odoRight, odoCenter;


//yurr b
    //rebase test


    @Override
    public void initialize() {
        // Initialization
        initializeHardware();
        initializeOdometry();
        odometry.updatePose(robotPose);

        waitForStart();
        while (opModeIsActive()) {
            // Autonomous actions
            // Update odometry
            mPath.followPath(driveTrain, odometry);
            telemetry.update();


            ppCommand.schedule(); // Schedule the command
            mPath.init();

            odometry.updatePose();
            robotPose = odometry.getPose();
            telemetry.addData("Robot Position: ", robotPose);
            telemetry.update();
        }
    }


    private void initializeHardware() {
        // Initialize motors
        // Hardware components
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");
        frontRight = new MotorEx(hardwareMap, "frontRight");


        intake = new Motor(hardwareMap, "intake");
        lifty = new Motor(hardwareMap, "lifty");
        // Initialize drive train
        driveTrain = new MecanumDrive(frontLeft, backLeft, backRight, frontRight);


        odoLeft = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        odoRight = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        odoCenter = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odoRight.setDirection(Motor.Direction.REVERSE);


        odometry = new HolonomicOdometry(
                odoLeft::getDistance,
                odoRight::getDistance,
                odoCenter::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET)
        // Set motor modes and other configurations


        // Motor locations for kinematics
        ;
        fRight = new Translation2d(-0.381, -0.381);
        bLeft = new Translation2d(0.381, 0.381);
        bRight = new Translation2d(-0.381, 0.381);


        // Creating kinematics object
        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                fLeft, fRight, bLeft, bRight);


        driveTrain.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);
    }


    private void initializeOdometry() {
        // Set distance per pulse for encoders
        double ticksToInches = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
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
            public void periodic () {
                Rotation2d gyroAngle = Rotation2d.fromDegrees(gyro.getAngle());
                odometry.updatePose();
                robotPose = new Pose2d(
                        robotPose.getTranslation(),
                        gyroAngle
                );
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
    }
}
*/