package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
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
    public static Pose2d robotPose;

    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive drivetrain;
    private Motor intake,lifty;
    //include intake elevation??  change to lifty
    private Encoder OdoLeft,OdoRight,OdoCenter;
    private HolonomicOdometry odometry;

    private Pose2d Pose;
    double getX;
    double getY;
    double getHeading;
    public void OdometrySubsystem(double intialX, double intialY) {
        this.Pose = new Pose2d();
    }
    public Pose2d getPose() {
        return Pose;
    }
    public double getX() {
        return getX;
    }
    public double getY() {
        return getY;
    }
    public double getHeading() {
        return getHeading;
    }
    @Override
    public void runOpMode() throws InterruptedException{
        frontLeft = new MotorEx(hardwareMap, "front_left");
        frontRight = new MotorEx(hardwareMap, "front_right");
        backLeft = new MotorEx(hardwareMap, "back_left");
        backRight = new MotorEx(hardwareMap, "back_right");

        drivetrain = new MecanumDrive(frontLeft,frontRight,backLeft,backRight);

        intake = new Motor(hardwareMap, "intake");
        lifty = new Motor(hardwareMap, "lifty");

        OdoLeft = frontLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        OdoRight = frontRight.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        OdoCenter = backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        OdoRight.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                OdoLeft::getDistance,
                OdoRight::getDistance,
                OdoCenter::getDistance,
                TRACKWIDTH,CENTER_WHEEL_OFFSET
        );


        waitForStart();

        while(opModeIsActive()&& !isStopRequested()) {
            Pose2d Pose = getPose();
            telemetry.addData("X Position (in)", Pose.getX());
            telemetry.addData("Y Position (in)", Pose.getY());
            telemetry.addData("Angle", Pose.getHeading());
            telemetry.update();
            odometry.updatePose();
            telemetry.update();

        }
    }
}