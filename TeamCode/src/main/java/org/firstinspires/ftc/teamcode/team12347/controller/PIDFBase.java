package org.firstinspires.ftc.teamcode.team7786.controller;

public class PIDFBase {
   
  public double kP, kI, kD, kF;                 // Store the tuning values
  private double currentValue, targetValue;      // Store current and target values
  private double totalError, lastError;         // Store total accumulated error (for integration) and last error (for differentiation)
  public double error, errorVelocity;           // Store error and error velocity (how fast the error changes)
  private double minIntegral, maxIntegral;                         // Store range limits for the integral to prevent overflow
  public double tolerance = 0.05;                                  // Defines how close is considered "close enough"
  private double toleranceVelocity = Double.POSITIVE_INFINITY;     // Defines maximum derivative gain tolerance. It is reccomended to keep this value at positive infinity.
  private double lastTime, period;                                 // Store the time the last iteration ran and the time between iterations.
  
  
  /** Constructor for class PIDF Base
   * @param kP              The proportional tuning value
   * @param kI              The integral tuning value
   * @param kD              The derivative tuning value
   * @param kF              The feedforward tuning value
   * @param targetValue     The target value for the object being controlled
   * @param minIntegral     The miminum bound for the integral
   * @param maxIntegral     The maximum bound for the integral
   */
  public PIDFBase (double kP, double kI, double kD, double kF, double targetValue, double minIntegral, double maxIntegral) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.targetValue = targetValue;
    this.currentValue = 0d;
    this.minIntegral = minIntegral;
    this.maxIntegral = maxIntegral;
    this.reset();
  }
  
   
  /** Calculates the output value for the controller
   * @param cV   the current value of the controller
   * @return     the output value for the controller
   */
  public double calculate (double cV) {
    // Calculate time since last execution
    double currentTime = (double) System.nanoTime() / 1E9;
    if (lastTime == 0) {
      lastTime = currentTime;
    }
    period = currentTime - lastTime;
    lastTime = currentTime;
    
    // Calculate error
    if (currentValue == cV) {
      error = targetValue - currentValue;
    } else {
      error = targetValue - cV;
      currentValue = cV;
    }
    
    // Calculate velocity error
    if (Math.abs(period) > 1E-6) {
      errorVelocity = (error - lastError) / period;
    } else {
      errorVelocity = 0;
    }
    
    // Calculate and bind integral
    totalError = period * (targetValue - currentValue);
    totalError = Math.max(Math.min(totalError, maxIntegral), minIntegral);
    
    // Return calculation
    return kP * error + kI * totalError + kD * errorVelocity + kF * targetValue;  
  }
  
  
  /* ********************************** ALL METHODS BEYOND THIS POINT ARE GETTERS/SETTERS*************************************** */
  /** Set the target value
   * @param targetValue    The new target value for the object being controlled
   */
  public void setTarget(double targetValue) {
    this.targetValue = targetValue;
  }
  
  /** Get the target value
   * @return     The current target value for the object being controlled
   */
  public double getTarget() {
    return targetValue;
  }
  
  
  /** Set the integration bounds
   * @param minIntegral    The minimum value of the integral
   * @param maxIntegral    The maximum value of the integral
   */
  public void setIntegralBounds(double minIntegral, double maxIntegral) {
    this.minIntegral = minIntegral;
    this.maxIntegral = maxIntegral;
  }
  
  /** Get the integration bounds
   * @return     A 2-item double array with {minIntegral, maxIntegral}
   */
  public double[] getIntegralBounds() {
    return new double[]{minIntegral, maxIntegral};
  }
  
  /** Clear error history (for integral calculation) */
  public void clearIntegral() {
    totalError = 0;
  }
  
  /** Clears all prior history (for integral and diferential calculations) */
  public void reset() {
    totalError = 0;
    lastError = 0;
    lastTime = 0;
  }
  
  
  /** Set the accepted tolerance
   * @param tolerance           How close to the target value is considered "close enough"
   * @param toleranceVelocity   How quickly the error can changed and still be considered "good enough"
   */
  public void setTolerance(double tolerance, double toleranceVelocity) {
    this.tolerance = tolerance;
    this.toleranceVelocity = toleranceVelocity;
  }
  
  /** get the accepted tolerance
   *  @return      A 2-item double array with {tolerance, toleranceVelocity}
   */
  public double[] getTolerance() {
    return new double[]{tolerance, toleranceVelocity};
  }
  
  
  /** Get the last measured value
   *  @return      the last measured value
   */
  public double lastMeasuredValue() {
    return currentValue;
  }
  
}
