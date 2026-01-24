package frc.lib.util;

public class Utilities {
    public static final double sparkFlexResolution = 7168;

    public static double polynomialAccleration(double x) {
        return Math.pow(x,3) * 0.795903 + x * 0.203938;
    }
    
    // keep drive values within the range of -1 and 1 
    public static double clampDriveValues(double driveInput) {
        driveInput = Math.min(1, Math.max(-1, driveInput));
        // Math.min(1,driveInput);
        // driveInput = Math.max(-1, driveInput);

        return driveInput; 
    }

    public static double convertGyroReadings(double reading) {
        double processedReading = reading % 360;
        if(processedReading > 180) {
            processedReading = processedReading - 360;
        }
        return processedReading;
    }

    public static double processYaw(double yaw) {
        if(yaw < 0) {
            return 360 - (Math.abs(yaw) % 360);
        } else {
            return yaw % 360;
        }

    }
}