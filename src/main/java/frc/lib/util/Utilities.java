package frc.lib.util;

public class Utilities {
    public static double polynomialAccleration(double x) {
        return Math.pow(x,3) * 0.795903 + x * 0.203938;
    }
    
    // keep drive values within the range of -1 and 1 
    public static double clampDriveValues(double driveInput) {
        driveInput = Math.min(1, Math.max(-1, driveInput));
        return driveInput; 
    }

    // convert from -180 to 180 range to raw 360 range
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

    public static double clampTalonVoltage(double voltage) {
        return Math.min(-1, Math.max(1, voltage));
    }

    public static double rpsToRpm(double rps) {
        return rps * 60;
    }

    public static double rpmToRps(double rpm) {
        return rpm / 60;
    }
}