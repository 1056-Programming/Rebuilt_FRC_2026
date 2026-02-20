package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.robot.CalculateShooterSpeed;
import frc.robot.Constants;
import frc.robot.States.ShooterStates;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.VisionSubsystem; 

public class ShooterSubsystem extends SubsystemBase {
    // Different motors for each channel on the robot 
    private final TalonFX m_rightShooter;
    private final TalonFX m_middleShooter;
    private final TalonFX m_leftShooter; 

    // One backspin motor for all channels 
    private final SparkFlex m_leftBackSpin;
    private final SparkFlex m_rightBackSpin;

    // Shooter PID to maintain cconstants RPS 
    private final PIDController rightPID;
    private final PIDController middlePID;
    private final PIDController leftPID;

    // Backspin PID to maintain cconstants RPS 
    private final PIDController backSpinPID;

    // Current shooter state 
    private ShooterStates s_state; 

    private VisionSubsystem s_limeDist; 

    private PoseEstimate s_poseEstimate; 

    private double bottomLaunchSpeed; 
    private double shootLaunchSpeed;
    private double s_distance; 


    public ShooterSubsystem() {
        m_rightShooter = new TalonFX(Constants.Shooter.kRightShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kMiddleShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kLeftShootingID);

        rightPID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kShooterD); 
        middlePID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kShooterD); 
        leftPID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kShooterD);

        m_leftBackSpin = new SparkFlex(Constants.Shooter.kLeftBackspinID, MotorType.kBrushless);
        m_rightBackSpin = new SparkFlex(Constants.Shooter.kRightBackspinID, MotorType.kBrushless);

        backSpinPID = new PIDController(Constants.Shooter.kBackP, Constants.Shooter.kBackI, Constants.Shooter.kBackD); 

        s_poseEstimate = new PoseEstimate(); 

        // s_limeDist = new VisionSubsystem(); // unsure on where and how to get the drivetrain

        SparkFlexUtils.setSparkFlexBusUsage(m_leftBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, false);
        SparkFlexUtils.setSparkFlexBusUsage(m_rightBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, true);

        bottomLaunchSpeed = 0; 
        shootLaunchSpeed = 0;
        // Initalize shooter in STOP position
        setShooterState(ShooterStates.STOP);
        changePipeline(0); 

    }

    @Override
    public void periodic() {
        // TODO: Constantly read vision measuremnts and then calculate optimal shooting speed 
        // Only use this when target is visiable
        // Possible to use based on the odomtery reading of the robot 
        // NEW TODO: Have to calculate the set point for the back spin motors

        if (s_poseEstimate.tagCount >= 0 
            || s_poseEstimate.rawFiducials[0].ambiguity < Constants.Vision.ambiguityThreshold
            || s_poseEstimate.rawFiducials[0].distToCamera < Constants.Vision.distanceThreshold ) {
            
            // s_distance = s_limeDist.megaTag2(s_poseEstimate); 
        }

        m_rightShooter.set(
            rightPID.calculate(m_rightShooter.getVelocity().getValueAsDouble())
        );
        
        m_leftShooter.set(
            leftPID.calculate(m_leftShooter.getVelocity().getValueAsDouble())
        );

        m_rightShooter.set(
            leftPID.calculate(m_rightShooter.getVelocity().getValueAsDouble())
        );

        // Update dashboard data periodically
        setDashboardData();
    }

    // create functions to set all the speed 
    public void setBackSpinSpeed(double speed) {
        m_leftBackSpin.set(speed);
        m_rightBackSpin.set(speed);
    }   

    public void setAllShooterSpeed(double speed) {
        m_rightShooter.set(speed);
        m_middleShooter.set(speed);
        m_leftShooter.set(speed);
    }

    private void setShooterSetpoint(double desiredRPS) {
        rightPID.setSetpoint(desiredRPS);
        middlePID.setSetpoint(desiredRPS);
        leftPID.setSetpoint(desiredRPS);
    }

    private void setBackSetpoint(double desiredRPS) {
        backSpinPID.setSetpoint(desiredRPS);
    }

    private void setShooterState(ShooterStates state) {
        s_state = state; 

        // If using variable calculations set motor speed to optimal speed 
        if(state.equals(ShooterStates.VARIABLE_SHOOT)) {
            // TODO insert code to calculate variable shooting speed
            // SHould be based on Limlight vision calculations and distance to target
            double[] optimalShotsResult = CalculateShooterSpeed.calculateOptimalShot(
                s_distance, Constants.CalculateShooter.TARGET_HEIGHT);
            setBackSpinSpeed(optimalShotsResult[1]);
            setAllShooterSpeed(optimalShotsResult[0]);

            
            return;
        } 

        // Otherwise use setpoint based motor speeds 
        setBackSpinSpeed(state.backSpinRPS);
        setAllShooterSpeed(state.shootingRPS);
    }

    public void changePipeline(int pipelineIndex) {
        // Sets the pipeline index for the default "limelight" camera
        LimelightHelpers.setPipelineIndex("limelight", pipelineIndex);
    }
    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // TODO add data to dashboard for testing and debugging purposes
        SmartDashboard.putNumber("Right Shooter Motor Speed", m_rightShooter.get());
        SmartDashboard.putNumber("Middle Shooter Motor Speed", m_middleShooter.get());
        SmartDashboard.putNumber("Left Shooter Motor Speed", m_leftShooter.get());

        SmartDashboard.putNumber("Right Back Spin Motor Speed", m_rightBackSpin.get());
        SmartDashboard.putNumber("Left Back Spin Speed", m_leftBackSpin.get()); 
        
    }
}
