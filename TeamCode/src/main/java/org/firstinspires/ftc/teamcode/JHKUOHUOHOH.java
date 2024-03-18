package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class JHKUOHUOHOH {

    DcMotorEx front_left = (DcMotorEx) hardwareMap.get(DcMotor.class, "front_left");
    DcMotorEx front_right = (DcMotorEx)hardwareMap.get(DcMotor.class, "front_right");
    DcMotorEx back_left = (DcMotorEx)hardwareMap.get(DcMotor.class, "back_left");
    DcMotorEx back_right = (DcMotorEx)hardwareMap.get(DcMotor.class, "back_right");


    public class OdometrySubsystem extends SubsystemBase {
        public class Pose2d {
            public int getWheelPositions() {
                int wheelPositions = 0;
                return wheelPositions;
                wheelPositions = new MecanumDriveWheelPositions(
                       back_left.getCurrentPosition(), back_right.getCurrentPosition();
            }

            }
            }


    public Pose2d getInstance() {


        private class MecanumDriveWheelPositions {
            public MecanumDriveWheelPositions(Object distance, Object distance1, Object p2) {
            }
        }
    }
}


