package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class JHKUOHUOHOH extends LinearOpMode {

    DcMotorEx front_left;
    DcMotorEx front_right;
    DcMotorEx back_left;
    DcMotorEx back_right;

    double ticksPerRev = 1440;
    double wheelDiameter = 3.0;
    double gearRatio = 1.0;

    double ticksPerInch = ticksPerRev / (wheelDiameter * Math.PI * gearRatio);

    public class OdometrySubsystem extends SubsystemBase {
        double robotX;
        double robotY;

        public OdometrySubsystem(double initialX, double initialY) {
            this.robotX = initialX;
            this.robotY = initialY;
        }

        public void updatePosition() {
            double frontLeftPosition = front_left.getCurrentPosition() / ticksPerInch;
            double frontRightPosition = front_right.getCurrentPosition() / ticksPerInch;
            double backLeftPosition = back_left.getCurrentPosition() / ticksPerInch;
            double backRightPosition = back_right.getCurrentPosition() / ticksPerInch;

            double averageLateralPosition = (frontLeftPosition + backLeftPosition) / 2;
            double averageLongitudinalPosition = (frontLeftPosition + frontRightPosition) / 2;


            robotX += averageLateralPosition;
            robotY += averageLongitudinalPosition;
        }
    }


    OdometrySubsystem odometrySubsystem = new OdometrySubsystem(0, 0);


    private static final double TRACKWIDTH = 13.7;
    private static final double TICKS_TO_INCHES = 15.3;
    private static final double CENTER_WHEEL_OFFSET = 2.4;

    @Override
    public void runOpMode() {

        front_left = (DcMotorEx) hardwareMap.get(DcMotor.class, "front_left");
        front_right = (DcMotorEx) hardwareMap.get(DcMotor.class, "front_right");
        back_left = (DcMotorEx) hardwareMap.get(DcMotor.class, "back_left");
        back_right = (DcMotorEx) hardwareMap.get(DcMotor.class, "back_right");

        initializeHardware();
        initializeOdometry();


        waitForStart();


        while (opModeIsActive()) {

            odometrySubsystem.updatePosition();

            telemetry.addData("X Position (in)", odometrySubsystem.robotX);
            telemetry.addData("Y Position (in)", odometrySubsystem.robotY);
            telemetry.update();
        }
    }

    private void initializeHardware() {

        MotorEx frontLeft = new MotorEx(hardwareMap, "front_left");
        MotorEx frontRight = new MotorEx(hardwareMap, "front_right");
        MotorEx backLeft = new MotorEx(hardwareMap, "back_left");
        MotorEx backRight = new MotorEx(hardwareMap, "back_right");

        frontLeft.setDistancePerPulse(TICKS_TO_INCHES);
        frontRight.setDistancePerPulse(TICKS_TO_INCHES);
        backLeft.setDistancePerPulse(TICKS_TO_INCHES);
        backRight.setDistancePerPulse(TICKS_TO_INCHES);

    }

    private void initializeOdometry() {

        MotorEx encoderLeft = new MotorEx(hardwareMap, "left_encoder");
        MotorEx encoderRight = new MotorEx(hardwareMap, "right_encoder");
        MotorEx encoderPerp = new MotorEx(hardwareMap, "center_encoder");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        HolonomicOdometry holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        OdometrySubsystem odometry = new OdometrySubsystem(0, 0);
    }
}
