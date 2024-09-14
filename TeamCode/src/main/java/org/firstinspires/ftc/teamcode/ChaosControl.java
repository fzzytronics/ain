package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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


    **/
    @Autonomous
public class ChaosControl extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime(); // Declare timer as a class member
    @Override
    public void runOpMode() {
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
}