package org.firstinspires.ftc.teamcode.team12347.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.team12347.core.ROBOT_DATA.*;

import org.firstinspires.ftc.teamcode.team12347.kinematics.Odometer;
import org.firstinspires.ftc.teamcode.team12347.kinematics.ThreeWheelOdometry;

// This code serves as a base class for a 4 motor drivetrain
public class Robot
{

  // Declare Drive Motors and Array
  private DcMotor leftFrontDrive, rightRearDrive, rightFrontDrive, leftRearDrive;
  private DcMotor[] drivers;

  // Declare Odometers and Odometry
  private Odometer rightOdo, leftOdo, backOdo;
  protected ThreeWheelOdometry odometry;

  /** Constructor for base class robot
   * @param hwMap     the hardware map
   */
  public Robot(HardwareMap hwMap) {
      // Initialize the motor hardware variable
      leftFrontDrive = hwMap.get(DcMotor.class, LEFT_FRONT_NAME);
      leftRearDrive = hwMap.get(DcMotor.class, RIGHT_FRONT_NAME);
      rightFrontDrive = hwMap.get(DcMotor.class, LEFT_REAR_NAME);
      rightRearDrive = hwMap.get(DcMotor.class, RIGHT_REAR_NAME);

      // Initialize the motor array

      //this right here is causing just so many problem
      drivers = new DcMotor[]{leftFrontDrive, rightFrontDrive, leftRearDrive, rightRearDrive};

      // set motor orientations (positive power should move wheel clockwise)
      for (int i = 0; i < 4; i++) {
        if (MOTOR_INVERTED[i]) {
            drivers[i].setDirection(DcMotor.Direction.REVERSE);
        }
      }

      // Create Odometry
      createOdometry();
  }


  /** Provides power to all motors.
   * @param powers     Motor order is LF, RF, LR, RR.
   */
  public void tankDrive (double[] powers) {
    for (int i = 0; i < 4; i++) {
      drivers[i].setPower(powers[i]);
    }
  }


  /** Stops all motors */
  public void stop () {
    tankDrive(new double[]{0d, 0d, 0d, 0d});
  }

  /** Reduces the heading of the robot until it is between 0 and 2 pi (not including 2pi)*/
  public void reduceRotation() {
  	while (robotPose.theta >= Math.PI * 2) {
  		robotPose.rotate(Math.PI * -2);
  	}
  	while (robotPose.theta < 0) {
      robotPose.rotate(Math.PI * 2);
    }
  }

  private void createOdometry() {
      rightOdo = new Odometer(rightFrontDrive, COUNTS_PER_INCH);
      leftOdo = new Odometer(leftFrontDrive, COUNTS_PER_INCH);
      backOdo = new Odometer(rightRearDrive, COUNTS_PER_INCH);

      odometry = new ThreeWheelOdometry(rightOdo, leftOdo, backOdo, TRACK_WIDTH, BACK_OFFSET);
  }
}
