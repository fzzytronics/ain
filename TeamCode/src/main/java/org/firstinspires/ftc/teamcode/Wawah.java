package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

    private double calculateSpeedModifier(double seconds) {
     //testing testing one two three testing testing one two three
        if (timer.seconds() > 5) {
            return 0.75; // Reduce speed after 5 seconds
        } else {
            return 1.0;  // Full speed initially
        }
    }
}