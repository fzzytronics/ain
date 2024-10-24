package org.firstinspires.ftc.teamcode.team7786.geometry;


/**
* This represents a single 2 dimensional plane upon which points can exist
* @see org.firstinspires.ftc.teamcode.team7786.geometry.Point
* @author TheConverseEngineer
*/
public class Graph
{
  /** Method to find the point closest to point P on line AB
   * @param A     any point on line AB
   * @param B     any other point on line AB
   * @param P     the point P
   * @return      the point on line AB closest to point P
   */
  public static Point getClosestPoint(Point A, Point B, Point P) {
    Point AP = P.subtractPoint(A);
    Point AB = B.subtractPoint(A);

    double AB2 = AB.x * AB.x + AB.y * AB.y;
    double AP_AB = AP.x * AB.x + AP.y * AB.y;
    double t = AP_AB / AB2;

    return A.addToPoint(AB.multiplyScalar(t));
  }


  /** Method to find all intersection points between line AB and circle C
   * @param A        any point on line AB
   * @param B        any other point on line AB
   * @param C        the center of circle C
   * @param radius   the radius of circle C
   * @return         An array of all intersection points between line AB and circle C, if the line does not intersect, returns an empty list
   */
  public static Point[] getCircleLineIntersectionPoints(Point A, Point B, Point C, double radius) {
    Point BA = B.subtractPoint(A);
    Point CA = C.subtractPoint(A);

    double a = BA.x * BA.x + BA.y * BA.y;
    double bBy2 = BA.x * CA.x + BA.y * CA.y;
    double c = CA.x * CA.x + CA.y * CA.y - radius * radius;

    double pBy2 = bBy2 / a;
    double q = c / a;

    double disc = pBy2 * pBy2 - q;

    if (disc < 0) {
        return new Point[0];
    }

    // if disc == 0 ... dealt with later
    double tmpSqrt = Math.sqrt(disc);
    double abScalingFactor1 = -pBy2 + tmpSqrt;
    double abScalingFactor2 = -pBy2 - tmpSqrt;

    Point p1 = new Point(A.x - (BA.x * abScalingFactor1), A.y - (BA.y * abScalingFactor1));

    if (disc == 0) { // abScalingFactor1 == abScalingFactor2
        return new Point[]{p1};
    }

    Point p2 = new Point(A.x - (BA.x * abScalingFactor2), A.y- (BA.y * abScalingFactor2));

    return new Point[]{p1, p2};
  }


}
