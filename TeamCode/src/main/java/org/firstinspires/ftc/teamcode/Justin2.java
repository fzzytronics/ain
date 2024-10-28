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
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;

@Autonomous
public class Justin2 extends LinearOpMode {
    public static final double TRACKWIDTH = 13.7;
    public static final double CENTER_WHEEL_OFFSET = 2.4;
    public static final double WHEEL_DIAMETER = 0.075;
    public static final double TICKS_PER_REV = 15.3;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private ServoEx gyro;
    public Translation2d fLeft, fRight, bLeft, bRight;
    public MecanumDriveKinematics kinematics;
    private MecanumDrive drivetrain;
    //private Motor intake, lifty;
    public Pose2d robotPose;
    private Encoder odoLeft, odoRight, odoCenter;
    public HolonomicOdometry odometry;

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
            odometry.updatePose();
            robotPose = odometry.getPose();
            telemetry.addData("Robot Position: ", robotPose);
            telemetry.update();
        }
    }

    public void periodic() {
        /**
         * GOAL - get gyro rotation and update it cont. in odometry portion
         * current issue  - declaring .getPosition
         * getPosition = (distance and angle) /We need distance is my guess(?)
         *
         * Yep, chanced odo to .getDistance to mitigate problem, idk how that would transfer to live testing tho
         * Also, gyroAngle is supposed to be a double here, but it's flagging it as a different type, not sure if
         * we just need to declare or if there's smth else going on cuz this is not a problem in Justin -P
         *
         * UPDATE: figured out that the error isnt gyroAngle itself, but rather that update.Pose can't handle >3 arguments
         *
         * UPDATE: gyroAngle is no longer in the odometry.update and has been moved to a seperate call for robotPose
         * it fixed the problem of updatePose only accepting three doubles and nothing more, ready for live test 7/11/24. -P
         *
         * UPDATE: JUSTIN LIVES!!! Just Telemetry of current position for now -P 8/22/24
         */
        Rotation2d gyroAngle = Rotation2d.fromDegrees(gyro.getAngle());
        odometry.updatePose();
        robotPose = new Pose2d(
                robotPose.getTranslation(),
                gyroAngle
        );
    }
}
