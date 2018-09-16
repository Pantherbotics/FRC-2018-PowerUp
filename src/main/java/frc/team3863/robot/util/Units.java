package frc.team3863.robot.util;

/** This is a helper class to convert from Talon Native Units to human-readable units, including Feet per Second.
 */
public class Units {

    private static final double TALON_TO_FPS_CONVERSION = .03067961572265625;           //( 1 rev/ 512 ticks) * (0.5pi ft/ 1 rev) * (10 [100ms] / 1 s)
    private static final double FPS_TO_TALON_CONVERSION = 1/TALON_TO_FPS_CONVERSION;
    private static final double TALON_TO_FEET_CONVERSION = .003067961;
    private static final double FEET_TO_TALON_CONVERSION = (1.0/TALON_TO_FEET_CONVERSION);
    public static double FPSToTalonNative(double fps) {
        return fps * FPS_TO_TALON_CONVERSION;
    }

    public static double TalonNativeToFPS(double nativeUnits) {
        return nativeUnits * TALON_TO_FPS_CONVERSION;
    }

    public static double TalonNativeToFeet(double nativeUnits){
        return nativeUnits * TALON_TO_FEET_CONVERSION;
    }

    public static double FeetToTalonNative(double feet){
        return feet * FEET_TO_TALON_CONVERSION;
    }
    /*
    public static void main(String[] args){
        System.out.println(TalonNativeToFPS(770));
        System.out.println(FPSToTalonNative(7));
    }*/

}
