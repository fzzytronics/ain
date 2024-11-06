//qiwi's miserable attempt at being a competent programmer
//pathing code
package org.firstinspires.ftc.teamcode.team12347;


import com.arcrobotics.ftclib.command.CommandOpMode;
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

@Autonomous(name = "Dylan")
public class Dylan extends LinearOpMode {

    private static final double TRACKWIDTH = 13.7;
    private static final double WHEEL_DIAMETER = 0.075;
    private static final double CENTER_WHEEL_OFFSET = 2.4;
    private static final double TICKS_PER_REV = 15.3;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private HolonomicOdometry odometry;
    private MecanumDrive driveTrain;
    private Path mPath;
    private double strafeSpeed;
    private double forwardSpeed;
    private double turn;
    private double heading;
    private PurePursuitCommand ppCommand;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private ServoEx gyro;
    public Translation2d fLeft, fRight, bLeft, bRight;
    public MecanumDriveKinematics kinematics;
    public Pose2d robotPose;
    private Motor intake, lifty;
    private Encoder odoLeft, odoRight, odoCenter;

    @Override
    public void runOpMode(){
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");

        // Initialize the positions for the translations
        fLeft = new Translation2d(0.381, -0.381);
        fRight = new Translation2d(-0.381, -0.381);
        bLeft = new Translation2d(0.381, 0.381);
        bRight = new Translation2d(-0.381, 0.381);

        MecanumDrive drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        //intake = new Motor(hardwareMap, "intake");
        //lifty = new Motor(hardwareMap, "lifty");

        kinematics = new MecanumDriveKinematics(fLeft, fRight, bLeft, bRight);

        odoLeft = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        odoRight = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        odoCenter = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odoRight.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                odoLeft::getDistance,
                odoRight::getDistance,
                odoCenter::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        robotPose = new Pose2d();
        //updated: robotPose as a Pose2d() to avoid confusion and null shennanigans -P
        odometry.updatePose(robotPose);

        telemetry.addData("Robot Position at Init: ", robotPose);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
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
}