package org.firstinspires.ftc.teamcode.team7786.utils;


public class MathFunctions {
    /** Small utility function that clips a double between two values
     * @param x         the value to check
     * @param min       the minimum value x can be
     * @param max       the maximum value x can be
     * @return          the clipped x
     */
    public static double clipRange(double x, double min, double max) {
        return Math.min(Math.max(x, min), max);
    }


    /** Overload of clipRange (previous) for increased conveniance. Assumes min to be -1 and max to be 1.
     * @param x         the value to check
     * @return          the clipped c
     */
    public static double clipRange(double x) {
        return clipRange(x, -1d, 1d);
    }


    /** Square magnitude of number while keeping the sign.
     * @param input      the number to square
     * @return           the number squared*/
    public static double squareInput(double input) {
        return input * Math.abs(input);
    }

    /** Utility function to normalize a double array
     * This function takes an array and scales it down until the largest value is equal to the magnitude
     * @param x          the array of doubles to scale
     * @param magnitude  the largest value that an item in x can be
     */
    public static void normalize(double[] x, double magnitude) {
        // The variable maxMagnitude signifies the largest absolute value in the array
        double maxMagnitude = Math.abs(x[0]);
        for (int i = 1; i < x.length; i++) {
            double temp = Math.abs(x[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }

        maxMagnitude = (maxMagnitude == 0) ? 1d : maxMagnitude;

        // Normalize each unit in the array with the maxMagnitude
        for (int i = 0; i < x.length; i++) {
            x[i] = (x[i] / maxMagnitude) * magnitude;
        }
    }
}