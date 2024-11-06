package org.firstinspires.ftc.teamcode.team7786.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.team7786.opmodes.gamepadconfigs.SampleConfig;


@TeleOp
public class Test1 extends OpMode
{
    DcMotorEx dave;
    double velocity;

    SampleConfig gamepad;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        velocity = 0;
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        dave  = hardwareMap.get(DcMotorEx.class, "dave");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        dave.setDirection(DcMotorEx.Direction.FORWARD);dave.setTargetPosition(0);
        dave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        dave.setMotorEnable();
        gamepad = new SampleConfig(gamepad1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double davePower;
        double drive = gamepad.left_stick_y();
        if (gamepad.a()){
            telemetry.addData("A button", gamepad.a());
        }
        if (gamepad.dpad_up()) {
            velocity += 1;
        }
        else if (gamepad.dpad_down()){
            if (velocity >= 1){
                velocity -= 1;
            }
        }

        dave.setPower(1);
        dave.setVelocity(velocity, AngleUnit.DEGREES);
        telemetry.addData("Velocity: ", velocity);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        
    }

}
