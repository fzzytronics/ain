package org.firstinspires.ftc.teamcode;

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

@Autonomous
public class Wawah extends LinearOpMode {
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

         /*     not 100% sure what to do here yet, but AI deems it necessary
            if (robot.followingPath) {
                robot.goToPosLinear();
            } else {
                Do something else if not following path*/
            }
        }
    /*public static double yPosB(double time) {
        double yPosedB;
        //interpolated thing below...later..
        yPosedB = bruh*bruh;
        double yPosedB1 = yPosedB;
        return yPosedB1;
        //This might have some logic issues, double yPosedB could be handled better, but it's fine for now.
    }

    public static double yDeriveB(double yPosed, double t) {
        t = bruh;
        yPosed = yPosedB;

        double ySpeed = (yPoseB(t+0.01)-yPosedB) *100;

        ySpeedB = Math.abs(ySpeed);
        return ySpeedB;
    } */
    private double calculateSpeedModifier(double seconds) {
     //testing testing one two three testing testing one two three
        if (timer.seconds() > 5) {
            return 0.75; // Reduce speed after 5 seconds
        } else {
            return 1.0;  // Full speed initially
        }
    }
}