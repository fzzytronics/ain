package org.firstinspires.ftc.teamcode.team12347;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Bottomright", group="LinearOpmode")
public class Bottomright extends LinearOpMode {


    private MotorEx frontLeft, frontRight, backLeft, backRight;

    private int frontLeftpos;
    private int frontrightpos;
    private int backleftpos;
    private int backrightpos;


    private static final int POSITION_TOLERANCE = 10;

    @Override
    public void runOpMode() {

        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        backRight = new MotorEx(hardwareMap, "backRight");


        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);


        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();


        frontLeftpos = 0;
        frontrightpos = 0;
        backrightpos = 0;
        backleftpos = 0;


        sleep(1000);

        waitForStart();


        drive(1650, 1650, 1650, 1650, 0.3);
        sleep(200);
    }


    public void drive(int frontlefttarget, int frontrighttarget, int backlefttarget, int backrighttarget, double speed) {

        frontLeftpos += frontlefttarget;
        frontrightpos += frontrighttarget;
        backleftpos += backlefttarget;
        backrightpos += backrighttarget;


        frontLeft.setTargetPosition(frontLeftpos);
        frontRight.setTargetPosition(frontrightpos);
        backLeft.setTargetPosition(backleftpos);
        backRight.setTargetPosition(backrightpos);


        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);


        frontLeft.set(speed);
        frontRight.set(speed);
        backLeft.set(speed);
        backRight.set(speed);


        while (opModeIsActive() &&
                Math.abs(frontLeft.getCurrentPosition() - frontLeftpos) > POSITION_TOLERANCE &&
                Math.abs(frontRight.getCurrentPosition() - frontrightpos) > POSITION_TOLERANCE &&
                Math.abs(backLeft.getCurrentPosition() - backleftpos) > POSITION_TOLERANCE &&
                Math.abs(backRight.getCurrentPosition() - backrightpos) > POSITION_TOLERANCE) {
            idle();
        }
    }
}
