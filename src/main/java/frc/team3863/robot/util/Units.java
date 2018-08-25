package frc.team3863.robot.util;

import frc.team3863.robot.Constants;

public class Units {
    public static double FPSToTalonNative (double fps){
        double nativeUnits = (fps * 12 * 1 * 4 * Constants.DRIVE_ENCODER_TICKS * 1)/(1 * 1 * 6 * Math.PI * 1 * 10);
        return nativeUnits;
    }

    public static double TalonNativeToFPS(double nativeUnits){
        double fps = (nativeUnits * 1 * Constants.DRIVE_WHEEL_DIAMETER * Math.PI * 10)/(1 * 4 * Constants.DRIVE_ENCODER_TICKS * 1 * 12 * 1);
        return fps;
    }

}
