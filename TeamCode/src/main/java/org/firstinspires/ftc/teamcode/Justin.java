package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;



@Autonomous
    public class Justin extends LinearOpMode {
    public static final double TRACKWIDTH = 13.7;
    public static final double CENTER_WHEEL_OFFSET = 2.4;
    public static final double WHEEL_DIAMETER = 0.075;
    public static final double TICKS_PER_REV = 15.3;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private ServoEx gyro;
    private Translation2d fLeft;
    private Translation2d fRight;
    private Translation2d bLeft;
    private Translation2d bRight;
    private SwerveDriveKinematics kinematics;
    private MecanumDrive drivetrain;
    private Motor intake,lifty;
    private Pose2d Pose2d;
    private Encoder OdoLeft,OdoRight,OdoCenter;
    private HolonomicOdometry odometry;
    @Override

    public void runOpMode(){
        frontLeft = new MotorEx(hardwareMap, "front_left");
        frontRight = new MotorEx(hardwareMap, "front_right");
        backLeft = new MotorEx(hardwareMap, "back_left");
        backRight = new MotorEx(hardwareMap, "back_right");

        //NOTE - add robot center in parenthesis (quad 1, 4, 2, 3)
        fLeft = new Translation2d(0.381,-0.381);
        fRight = new Translation2d();
        bLeft = new Translation2d();
        bRight = new Translation2d();

        drivetrain = new MecanumDrive(frontLeft,frontRight,backLeft,backRight);

        intake = new Motor(hardwareMap, "intake");
        lifty = new Motor(hardwareMap, "lifty");

        kinematics = new SwerveDriveKinematics(
                fLeft, fRight, bLeft, bRight
        );

        OdoLeft = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        OdoRight = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        OdoCenter = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        OdoRight.setDirection(Motor.Direction.REVERSE);

        //trying to fix gyro angle, how will it effect /odometry()/
        odometry = new HolonomicOdometry(
                OdoLeft::getDistance,
                OdoRight::getDistance,
                OdoCenter::getDistance,
                TRACKWIDTH,CENTER_WHEEL_OFFSET
        );
        odometry.updatePose(PositionTracker.robotPose);

        telemetry.addData("Robot Position at Init: ", (PositionTracker.robotPose));
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();
            telemetry.update();
        }
    }
    public void periodic(){
        //GOAL - get gyro rotation and update it cont. in odometry portion
        //current issue  - declaring .getPosition
        //getPosition = (distance and angle) /We need distance is my guess(?)
        Rotation2d gyroAngle = Rotation2d.fromDegrees(gyro.getAngle());
        Pose2d = odometry.update(gyroAngle,
                new SwerveDriveKinematics(
                        fLeft.getPosition(), fRight.getPosition(),
                        bLeft.getPosition(), bRight.getPosition()
                ));
    }
}