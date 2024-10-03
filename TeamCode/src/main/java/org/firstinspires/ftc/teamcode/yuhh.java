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
public class yuhh extends LinearOpMode {
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");

        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontRight.set(0.37);    // 37% of maximum speed in current direction

        backRight.setRunMode(Motor.RunMode.RawPower);
        backRight.set(0.37);    // 37% of maximum speed in current direction

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontLeft.set(0.37);    // 37% of maximum speed in current direction

        backLeft.setRunMode(Motor.RunMode.RawPower);
        backLeft.set(0.37);    // 37% of maximum speed in current direction

    }
}