package org.firstinspires.ftc.teamcode.team12347.core;

import static org.firstinspires.ftc.teamcode.team12347.core.ROBOT_DATA.*;
import static org.firstinspires.ftc.teamcode.team12347.utils.MathFunctions.clipRange;
import static org.firstinspires.ftc.teamcode.team12347.utils.MathFunctions.squareInput;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.team12347.controller.PIDFBase;
import org.firstinspires.ftc.teamcode.team12347.geometry.Point;
import org.firstinspires.ftc.teamcode.team12347.kinematics.MecanumDriveQueue;


public class MecanumRobot extends Robot
{

  public MecanumRobot(HardwareMap hwMap) {
    super(hwMap);

  }

  /** Updates the robot Pose and location of the robot using only the odometry encoders */
  protected void updateOdometry() {
  	robotPose.addPose(odometry.getDeltaPose());
  	reduceRotation();
  }

  /** Drives the robot in a specified direction
   * @param theta       the direction the robot should travel
   * @param speed       the speed at which the robot should travel
   * @param turnSpeed   the speed at which the robot should turn whilst moving
   */
  public void driveWithHeading(double theta, double speed, double turnSpeed) {
  	turnSpeed = clipRange(turnSpeed);
  	speed = clipRange(speed);

	// Create new mecanum drive queue
  	MecanumDriveQueue queue = new MecanumDriveQueue();

	// add lateral movement
	queue.addVectorMovement(theta, speed);

	// add turning
	queue.addRotationalMovement(turnSpeed);

	// Apply Movement
	tankDrive(queue.getMovements());
  }


  /** Drives the robot using parameters that can be inputted from a controller
   * @param x                  the x value of the drive joystick
   * @param y                  the y value of the drive joystick
   * @param rotX               the x value of the turn joystick
   * @param fieldCentric       setting this to true will drive the robot relative to the field (recommended)
   */
  public void driveWithController(double x, double y, double rotX, boolean fieldCentric){
    double squareX = clipRange(squareInput(x));
    double squareY = clipRange(squareInput(y));
    double squareRotX = clipRange(squareInput(rotX));

    double heading = fieldCentric ? Math.atan2(squareX, squareY) - robotPose.theta : Math.atan2(squareX, squareY);
    double speed = Math.hypot(squareX, squareY);
    driveWithHeading(heading, speed, squareRotX);
  }


  /** Drives the robot towards a specified point on the field
   * @param x             the x value of the point
   * @param y             the y value of the point
   * @param targetHeading the heading that the robot should be at
   * @param speed         the speed at which the robot should move
   */
  public void driveTowardsPoint(double x, double y, double targetHeading, double speed) {
  	double deltaX = x - robotPose.x;
  	double deltaY = y - robotPose.y;

  	double heading = Math.atan2(deltaX, deltaY);
  	driveWithHeading(heading, speed, GET_ROTATIONAL_SPEED(robotPose.theta, targetHeading));
  }

  /** Drives the robot towards a specified point on the field
   * @param target  the point to pursue
   * @param speed   the speed at which the robot should move
   */
  public void pursuePoint(Point target, double speed) {
    double deltaX = target.x - robotPose.x;
    double deltaY = target.y - robotPose.y;

    double heading = Math.atan2(deltaX, deltaY);
    driveWithHeading(heading, speed, GET_ROTATIONAL_SPEED(robotPose.theta, heading));
  }

}
