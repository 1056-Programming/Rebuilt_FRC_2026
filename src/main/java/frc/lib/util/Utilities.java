package frc.lib.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    public static double convertYawReadings(double reading) {
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

    public static double calculateYawToCenterPiece(double x, double y) {
      //  if(DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            var yaw = Math.atan((4 - y)/(4.6 - x));
            return Units.radiansToDegrees(yaw);
        // }
        // return 0; 
    }

    public static double calculateDistanceToCenterPiece(double x, double y) {
        return Math.pow((Math.pow(4.6 - x,2) + Math.pow(4- y,2)), 0.5);
    }

    public static double calculcateBackSpinSpeed(double distance) {
        return 16 * Math.pow(distance, 3) - 48 * Math.pow(distance, 2) + 40 * distance + 24;
    }

    public static double calculateShooterSpeed(double distance) {
        return 40 / (1 + Math.pow(Math.E, -(41.11991 * distance - 20.45987)));
    }
}