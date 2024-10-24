package org.firstinspires.ftc.teamcode.team7786.kinematics;

public class MecanumDriveQueue {
    private double[] motorValues;

    /** Creates a new Mecanum Drive Queue */
    public MecanumDriveQueue () {
        motorValues = new double[]{0d, 0d, 0d, 0d};
    };

    public void addVectorMovement(double theta, double speed) {
        motorValues[0] += Math.sin(theta + Math.PI / 4);
        motorValues[1] += Math.sin(theta - Math.PI / 4);
        motorValues[2] += Math.sin(theta - Math.PI / 4);
        motorValues[3] += Math.sin(theta + Math.PI / 4);

        normalizeMotorValues(speed);
    }

    public void addRotationalMovement(double turnSpeed) {
        double motorLimit = getMotorNormalizationLimit();

        motorValues[0] += turnSpeed;
        motorValues[1] -= turnSpeed;
        motorValues[2] += turnSpeed;
        motorValues[3] -= turnSpeed;

        normalizeMotorValues(motorLimit);
    }

    private void normalizeMotorValues (double max) {
        double normalizeValue = getMotorNormalizationLimit();
        for (double i : motorValues) {
            i = (i / normalizeValue) * Math.abs(max);
        }
    }

    private double getMotorNormalizationLimit () {
        return Math.max(Math.max(Math.abs(motorValues[0]), Math.abs(motorValues[1])),
                Math.max(Math.abs(motorValues[2]), Math.abs(motorValues[3])));
    }
    public double[] getMovements() {
        return motorValues;
    }
}
