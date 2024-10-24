package org.firstinspires.ftc.teamcode.team7786.core;

import org.firstinspires.ftc.teamcode.team7786.geometry.Pose;
import org.firstinspires.ftc.teamcode.team7786.utils.MathFunctions;
import org.firstinspires.ftc.teamcode.team7786.utils.MathFunctions;

public class ROBOT_DATA
{
  /****************************** CONSTANTS ******************************/
  //Software Version
  public static final String VERSION = "0.0.1";


  // Hardware map names
  public static final String LEFT_FRONT_NAME = "leftFrontDrive";
  public static final String RIGHT_FRONT_NAME= "rightFrontDrive";
  public static final String LEFT_REAR_NAME = "leftRearDrive";
  public static final String RIGHT_REAR_NAME = "rightRearDrive";

  // Drivetrain inversions
  public static final boolean[] MOTOR_INVERTED = {false, true, false, true};

  // The number of encoder counts in one inch of travel of an odometry pod.
  public static final int COUNTS_PER_INCH = 2048;

  // The number of inches between the left and right Odometry pod.
  public static final double TRACK_WIDTH = 6d;

  // The distance in inches between the back odometry wheel and the robot center.
  public static final double BACK_OFFSET = 6d;

  // The tuning function for autonomous robot rotation
  public static double GET_ROTATIONAL_SPEED(double heading, double target) {
    return MathFunctions.clipRange(target - heading, -1d, 1d);
  }

  // The tuning function for beginning autonomous robot rotation
  public static double GET_ROTATIONAL_START_POINT (double deltaHeading) {
    return deltaHeading;
  }


  // The maximum allowable range between the robot center and the line it is trying to follow in inches
  public static final double MAXIMUM_FOLLOW_RANGE = 5d;

  // The inaccuracy offset to prevent indeterminate forms in inches
  public static final double INDETERMINANT_OFFSET = 0.001d;

  // The default pure pursuit line follow radius in inches
  public static final double DEFAULT_FOLLOW_RADIUS = 5d;

  // The margin of error allowable by robot rotations in radians
  public static final double ROTATIONAL_ERROR_MARGIN = Math.toRadians(3);




  /****************************** STATES - DO NOT EDIT ******************************/

  // Robot positioning
  public static Pose robotPose = new Pose(0d, 0d, 0d);


}
