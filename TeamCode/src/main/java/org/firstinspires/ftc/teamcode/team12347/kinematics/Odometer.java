package org.firstinspires.ftc.teamcode.team12347.kinematics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Odometer {

    private DcMotor encoder;
    private int tpi;

    private int offSet;
    private int value;

    /** Creates a odometer object from a DcMotor
     * @param encoder   the motor port that this odometer is attached to
     * @param tpi       the ticks per revolution of this encoder*/
    public Odometer (DcMotor encoder, int tpi) {
        this.encoder = encoder;
        this.tpi = tpi;
        offSet = 0;
    }

    /** Creates a odometer object DcMotorEx
     * @param encoder   the motorEX port that this odometer is attached to
     * @param tpi       the ticks per revolution of this encoder*/
    public Odometer (DcMotorEx encoder, int tpi) {
        this.encoder = encoder;
        this.tpi = tpi;
        offSet = 0;
    }

    /** Returns the offset of this encoder
     * @return   the offset of this encoder*/
    public int getOffset() {
        return offSet;
    }

    /** Returns the value of this encoder in inches
     * @return   the value of this encoder in inches*/
    public double getValue () {
        value = encoder.getCurrentPosition() + offSet;
        return (double)value / tpi;
    }

    /** Returns the raw value of this encoder in inches
     * @return   the raw value of this encoder*/
    public int getRawValue () {
        value = encoder.getCurrentPosition() + offSet;
        return value;
    }

    /** Returns the global raw value of this encoder
     * @return   the global raw value of this encoder*/
    public int getGlobalValue () {
        value = encoder.getCurrentPosition();
        return value;
    }
    
    /** Resets the encoder to 0 without needing to stop*/
    public void resetEncoder() {
        offSet = -getRawValue();
    }
    
    /** Returns the last read value of this encoder. DOES NOT SEND A LYNX COMMAND 
     * @return    the last read value */
    public int ghostRead() {
        return value;
    }
}
