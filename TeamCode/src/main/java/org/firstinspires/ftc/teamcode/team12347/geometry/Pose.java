package org.firstinspires.ftc.teamcode.team7786.geometry;

public class Pose extends Point {
    public double theta;

    /** Constructor for class Pose
     * @param x      the x value
     * @param y      the y value
     * @param theta  the heading (radians)
     * */
    public Pose(double x, double y, double theta) {
        super(x, y);
        this.theta = theta;
    }

    /** Creates a new Pose from a double array
     * @param poseInfo    double array: [x, y, theta]
     * @return            the created pose
     */
    public static Pose createPoseFromArray(double[] poseInfo) {
        return new Pose(poseInfo[0], poseInfo[1], poseInfo[2]);
    }

    /** Rotates the pose for a specified number of radians
     * @param rotation    the number of radians to rotate the pose by
     */
    public void rotate(double rotation) {
        theta = theta + rotation;
    }

    /** Returns the heading in degrees
     * @return     the heading in degrees
     */
    public double getThetaDegrees() {
        return Math.toDegrees(theta);
    }

    public void addPose(Pose pose) {
        this.x = this.x + pose.x;
        this.y = this.y + pose.y;
        this.theta = this.theta + pose.theta;
    }
}
