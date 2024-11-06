package org.firstinspires.ftc.teamcode.team12347.kinematics;


import org.firstinspires.ftc.teamcode.team12347.geometry.Pose;

public class ThreeWheelOdometry {

    public Odometer rightOdo;
    public Odometer leftOdo;
    public Odometer backOdo;
    private double trackWidth;
    private double backOffset;

    private double lastRight;
    private double lastLeft;
    private double lastBack;

    // The order is R, L, B
    private double[] values = new double[3];
    private double[] deltas = new double[3];

    public ThreeWheelOdometry(Odometer rightOdo, Odometer leftOdo, Odometer backOdo, double trackWidth, double backOffset) {
        this.rightOdo = rightOdo;
        this.leftOdo = leftOdo;
        this.backOdo = backOdo;
        this.trackWidth = trackWidth;
        this.backOffset = backOffset;
    }

    private double[] calculate() {
        updateEncoders();
        double[] deltaCoords;
        double deltaHeading = calcHeading();
        if (deltaHeading == 0) {
            deltaCoords = calcStraight();
        } else {
            deltaCoords = calcCurve(deltaHeading);
        }
        return new double[]{deltaCoords[0], deltaCoords[1], deltaHeading};
    }

    public Pose getDeltaPose() {
        return Pose.createPoseFromArray(calculate());
    }

    private void updateEncoders () {
        double[] temps = new double[]{rightOdo.getValue(), leftOdo.getValue(), backOdo.getValue()};
        for (int i = 0; i < 3; i++) {
            deltas[i] = temps[i] - values[i];
            values[i] = temps[i];
        }
    }

    private double calcHeading() {
        return (deltas[0] - deltas[1]) / (2 * trackWidth);
    }

    private double[] calcStraight() {
        double x = deltas[2];
        double y = (deltas[0] + deltas[1]) / 2;
        return new double[]{x, y};
    }

    private double[] calcCurve(double dH) {
        double rT = ( trackWidth * (deltas[0] + deltas[1]) ) / (deltas[0] - deltas[1]);
        double rS = (deltas[2] / dH) - backOffset;
        double x = rT * (Math.cos(dH) - 1) + (rS * Math.sin(dH));
        double y = rT * Math.sin(dH) + (rS * (1 - Math.cos(dH)));
        return new double[]{x, y};
    }


}
