package org.firstinspires.ftc.teamcode.team7786.geometry;

/**
* This represents a single point on a 2 dimensional plane
* @see org.firstinspires.ftc.teamcode.team7786.geometry.Graph
* @author TheConverseEngineer
*/
public class Point
{
  /**
  * The x coordinate of the point
  */
  public double x;

  /**
  * The y coordinate of the point
  */
  public double y;


  /**  Constructor for the point class
   * @param _x    the x value of the point
   * @param _y    the y value of the point
   */
  public Point(double _x, double _y) {
    x = _x;
    y = _y;
  }


  /**  Adds another point to this point
   * @param pointToAdd     the point that should be added to this point
   * @return               the sum
   */
  public Point addToPoint(Point pointToAdd) {
    double newX = x + pointToAdd.x;
    double newY = y + pointToAdd.y;
    return new Point(newX, newY);
  }


  /**  Subtracts another point from this point
   * @param pointToSubtract     the point that should be subtracted to this point
   * @return                    the difference
   */
  public Point subtractPoint(Point pointToSubtract) {
    double newX = x - pointToSubtract.x;
    double newY = y - pointToSubtract.y;
    return new Point(newX, newY);
  }


  /**  Subtracts this point from another point
   * @param pointToSubtract     the point that this point should be subtracted from
   * @return                    the difference
   */
  public Point subtractFromPoint(Point pointToSubtract) {
    double newX = pointToSubtract.x - x;
    double newY = pointToSubtract.y - y;
    return new Point(newX, newY);
  }


  /**  Multiplies a point by a double
   * @param a    the scalar that the point should be multiplied by
   * @return     the product
   */
  public Point multiplyScalar(double a) {
   return new Point(x * a, y * a);
  }


  /**  Finds the distance between two points
   * @param p   any other point
   * @return    the distance between this point and point P
   */
  public double distToPoint(Point p) {
    double deltaX =  Math.pow(p.x - x, 2);
    double deltaY =  Math.pow(p.y - y, 2);
    return Math.sqrt(deltaX + deltaY);
  }

}
