package org.firstinspires.ftc.teamcode.team12347.geometry;

public class Line {
    public Point A;
    public Point B;

    /** Constructs a line given two points
     * @param A    The first point on the line
     * @param B    The second point on the line
     */
    public Line(Point A, Point B) {
        this.A = A;
        this.B = B;
    }

    /** Constructs a line given the x and y coordinates of two points
     * @param aX    The x value of the first point
     * @param aY    The y value of the first point
     * @param bX    The x value of the second point
     * @param bY    The y value of the second point
     */
    public Line(double aX, double aY, double bX, double bY) {
        this.A = new Point(aX, aY);
        this.B = new Point(bX, bY);
    }

    /** Returns the slope of the line. Returns infinity if vertical
     * @return  the slope of the line
     */
    public double getSlope() {
        double rise = A.y - B.y;
        double run = A.x - B.x;
        if (run == 0) { return Double.POSITIVE_INFINITY; }
        else { return rise / run; }
    }

    public void fixIndeterminant (double offset) {
        if (A.y - B.y == 0) { // Horizontal
            B.y += offset;
        } else if (A.x - B.x == 0) { // Vertical
            B.x += offset;
        }
    }
}
