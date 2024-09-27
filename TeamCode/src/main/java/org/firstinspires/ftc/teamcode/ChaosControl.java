package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.Justin2;

    /** timer //
    we need map out coords ///
    justin = cord
            idk how speed (mulitplication mototrrrr)
    we need calculus

yippppepeee
    **/
    @Autonomous
public class ChaosControl extends LinearOpMode {
        public static final double TRACKWIDTH = 13.7;
        public static final double CENTER_WHEEL_OFFSET = 2.4;
        public static final double WHEEL_DIAMETER = 0.075;
        public static final double TICKS_PER_REV = 15.3;
        public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

        private MotorEx frontLeft, frontRight, backLeft, backRight;
        private ServoEx gyro;
        public Translation2d fLeft, fRight, bLeft, bRight;
        public SwerveDriveKinematics kinematics;
        private MecanumDrive drivetrain;
        private Motor intake, lifty;
        public Pose2d robotPose;
        private Encoder odoLeft, odoRight, odoCenter;
        public HolonomicOdometry odometry;



    private ElapsedTime timer = new ElapsedTime(); // Declare timer as a class member
    @Override
    public void runOpMode() {

        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");

        // Initialize the positions for the translations
        fLeft = new Translation2d(0.381, -0.381);
        fRight = new Translation2d(-0.381, -0.381);
        bLeft = new Translation2d(0.381, 0.381);
        bRight = new Translation2d(-0.381, 0.381);

        drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        intake = new Motor(hardwareMap, "intake");
        lifty = new Motor(hardwareMap, "lifty");

        kinematics = new SwerveDriveKinematics(fLeft, fRight, bLeft, bRight);

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
            odometry.updatePose();
            robotPose = odometry.getPose();
            telemetry.addData("Robot Position: ", robotPose);
            telemetry.update();
        }


        timer.reset(); // Start the timer at the beginning of Autonomous

        waitForStart();

        while (opModeIsActive()) {
            double seconds = timer.seconds();
            telemetry.addData("Time (seconds)", seconds);
            telemetry.update();
            double speedModifier = calculateSpeedModifier(seconds);
        }
    }

    private double calculateSpeedModifier(double seconds) {
        //testing testing one two three testing testing one two three
        if (timer.seconds() > 5) {
            return 0.75; // Reduce speed after 5 seconds
        } else {
            return 1.0;  // Full speed initially
        }
    }
    public void periodicPose() {
        Rotation2d gyroAngle = Rotation2d.fromDegrees(gyro.getAngle());
        odometry.updatePose();
        robotPose = new Pose2d(
                robotPose.getTranslation(),
                gyroAngle
        );

    }
}