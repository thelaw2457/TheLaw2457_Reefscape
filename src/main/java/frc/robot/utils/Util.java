package frc.robot.utils;

public class Util {

    public static boolean isWithinTolerance(double desiredValue, double realValue, double toleranceValue) {
        return Math.abs(desiredValue - realValue) >= toleranceValue;
    }
    
}