package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Utilities;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain; 
    private final boolean useMegaTag2;
    private LimelightHelpers.PoseEstimate s_poseEstimate; 
    
    private double limelight_tx; 
    private double robotYaw;
    public static double tag_distance; 

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        this.useMegaTag2 = false; // Set to true to use MegaTag2, false for AprilTag 

        this.robotYaw = 0; 

        LimelightHelpers.setPipelineIndex("limelight-hotrock", 0);
    }

    @Override
    public void periodic() {
        s_poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-hotrock");  
        megaTag2(s_poseEstimate);


        SmartDashboard.putNumber("Number of AprilTags", s_poseEstimate.tagCount);
        SmartDashboard.putNumber("Yaw of Robot", robotYaw);
    }

    private void megaTag1(PoseEstimate poseEstimate) {
        // No vaild april tags detected or pose estimate too unreliable 
        if(poseEstimate.tagCount == 0  
            || poseEstimate.rawFiducials[0].ambiguity > Constants.Vision.ambiguityThreshold
            || poseEstimate.rawFiducials[0].distToCamera > Constants.Vision.distanceThreshold) {
            return;
        } else if (poseEstimate.tagCount >= 1) {
            drivetrain.addVisionMeasurement(
                poseEstimate.pose, 
                poseEstimate.timestampSeconds,
                calculateStdDevs(poseEstimate.rawFiducials[0].distToCamera)
                );
            SmartDashboard.putNumber("Limelight Distance", poseEstimate.rawFiducials[0].distToCamera);
        }

        
    }

    public void megaTag2 (LimelightHelpers.PoseEstimate poseEstimate) {        
        if(poseEstimate.tagCount == 0  
            || poseEstimate.rawFiducials[0].ambiguity > Constants.Vision.ambiguityThreshold
            || poseEstimate.rawFiducials[0].distToCamera > Constants.Vision.distanceThreshold) {
            return;
        } else if (poseEstimate.tagCount == 1) {
            robotYaw = Utilities.convertGyroReadings(drivetrain.getPigeon2().getYaw().getValueAsDouble());
                    // find angular velocity later 

            // gets the horizontal, angular offset of the liemlight relative to the middle of the april tag
            limelight_tx = LimelightHelpers.getTX("limelight-hotrock"); 
            limelight_tx+=robotYaw; 
            LimelightHelpers.SetRobotOrientation("limelight-hotrock", limelight_tx, 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-hotrock");
            
            drivetrain.setVisionMeasurementStdDevs(calculateStdDevs(poseEstimate.rawFiducials[0].distToCamera));
            drivetrain.addVisionMeasurement(
                    poseEstimate.pose, 
                    poseEstimate.timestampSeconds
            );
            
            tag_distance = poseEstimate.rawFiducials[0].distToCamera;
            SmartDashboard.putNumber("Limelight Distance", tag_distance);
        }
         
    }

    private Matrix<N3, N1> calculateStdDevs(double distance) {
        // 1. Define your base trust (error in meters/radians when very close)
        double baseStdDevTrans = 0.05; // 5 centimeters
        double baseStdDevRot = 0.1;    // ~5.7 degrees

        // Adjust these based on your specific camera mounting height and light levels

        // Calculate the dynamic std dev using a quadratic curve (distance^2)
        double calculatedTrans = baseStdDevTrans + (0.24 * Math.pow(distance, 2));
        double calculatedRot = baseStdDevRot + (0.43 * Math.pow(distance, 2));

        // 4. Return the matrix for the Pose Estimator
        return VecBuilder.fill(calculatedTrans, calculatedTrans, calculatedRot);
    }


}
