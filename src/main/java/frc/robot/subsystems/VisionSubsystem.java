package frc.robot.subsystems;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
    private String limelightName; 
    
    private double limelight_tx; 
    private double robotYaw;
    public static double tag_distance; 

    public static SwerveDrivePoseEstimator swerveDrivePoseEstimator; 

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName){
        this.drivetrain = drivetrain;
        this.useMegaTag2 = false; // Set to true to use MegaTag2, false for AprilTag 
        this.limelightName = limelightName; 

        this.robotYaw = 0; 

        LimelightHelpers.setPipelineIndex(limelightName, 0);
    }

    @Override
    public void periodic() {
        s_poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);  
        megaTag2(s_poseEstimate);
        setSmartDashboard();
    }

    public void megaTag2 (LimelightHelpers.PoseEstimate poseEstimate) { 
        
        if(poseEstimate.tagCount == 0  
            || poseEstimate.rawFiducials[0].ambiguity > Constants.Vision.ambiguityThreshold
            || poseEstimate.rawFiducials[0].distToCamera > Constants.Vision.distanceThreshold) {
            return;
        } else if (poseEstimate.tagCount == 1) {
            robotYaw = Utilities.convertGyroReadings(drivetrain.getPigeon2().getYaw().getValueAsDouble());
            tag_distance = poseEstimate.rawFiducials[0].distToCamera;
                    // find angular velocity later 
            LimelightHelpers.SetRobotOrientation(limelightName, robotYaw, 0, 0, 0, 0, 0);     

            drivetrain.setVisionMeasurementStdDevs(calculateStdDevs(tag_distance));
            drivetrain.addVisionMeasurement(
                    poseEstimate.pose, 
                    poseEstimate.timestampSeconds,
                    calculateStdDevs(tag_distance)
            );
        }
        
    }

    public double getTagYaw() {
        return LimelightHelpers.getTX(limelightName); 
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

    private void setSmartDashboard() {

        // Get the yaw of the robot and the yaw from tag
        SmartDashboard.putNumber(limelightName+" Yaw from Tag", getTagYaw());
        SmartDashboard.putNumber("Yaw of Robot", robotYaw);

        // Get the number of apritags and the distance from the closest one
        SmartDashboard.putNumber(limelightName+" Number of AprilTags", s_poseEstimate.tagCount);
        SmartDashboard.putNumber(limelightName+" Distance", tag_distance);
    }


}
