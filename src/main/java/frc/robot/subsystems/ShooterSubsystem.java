package frc.robot.subsystems;

import java.lang.Thread.State;

import org.opencv.video.DenseOpticalFlow;

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

    private double rightPIDCalc;
    private double middlePIDCalc;
    private double leftPIDCalc; 

    private double desiredRPS; 


    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_rightShooter = new TalonFX(Constants.Shooter.kRightShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kMiddleShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kLeftShootingID);

        m_leftBackSpin = new SparkFlex(Constants.Shooter.kLeftBackspinID, MotorType.kBrushless);
        m_rightBackSpin = new SparkFlex(Constants.Shooter.kRightBackspinID, MotorType.kBrushless);

        rightPID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kBackD);
        leftPID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kShooterD);
        middlePID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kShooterD);

        backSpinPID = new PIDController(Constants.Shooter.kBackP, Constants.Shooter.kBackI, Constants.Shooter.kBackD); 

        s_limeDist = new VisionSubsystem(drivetrain); // unsure on where and how to get the drivetrain

        SparkFlexUtils.setSparkFlexBusUsage(m_leftBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, false);
        SparkFlexUtils.setSparkFlexBusUsage(m_rightBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, true);

        setShooterSetpoint(0);
        setBackSetpoint(0);

        // Initalize shooter in STOP position
        setShooterState(ShooterStates.STOP);
        LimelightHelpers.setPipelineIndex("limelight-hotrock", 0);
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-hotrock", Constants.Shooter.validIDs);

    }

    @Override
    public void periodic() {
        // TODO: Constantly read vision measuremnts and then calculate optimal shooting speed 
        // Only use this when target is visiable
        // Possible to use based on the odomtery reading of the robot 

        // NEW TODO: Have to calculate the set point for the back spin motors
        
        if (desiredRPS == 0) {
            setAllShooterSpeed(0);
            setBackSpinSpeed(0);
        } else if (Math.round(m_leftShooter.getVelocity().getValueAsDouble()) == Math.round(desiredRPS)
        || Math.round(m_middleShooter.getVelocity().getValueAsDouble()) == Math.round(desiredRPS) 
        || Math.round(m_rightShooter.getVelocity().getValueAsDouble()) == Math.round(desiredRPS)) {

            m_rightShooter.set(
                rightPIDCalc = rightPID.calculate(m_rightShooter.getVelocity().getValueAsDouble())
            );
            
            m_leftShooter.set(
                leftPIDCalc = leftPID.calculate(m_leftShooter.getVelocity().getValueAsDouble())
            );

            m_middleShooter.set(
                middlePIDCalc = middlePID.calculate(m_middleShooter.getVelocity().getValueAsDouble())
            );

            m_leftBackSpin.set(
                backSpinPID.calculate(m_leftBackSpin.get())
            );

            m_rightBackSpin.set(
                backSpinPID.calculate(m_rightBackSpin.get())
            );
        }


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

    public void setShooterState(ShooterStates state) {
        s_state = state; 

        // If using variable calculations set motor speed to optimal speed 
        if(state.equals(ShooterStates.VARIABLE_SHOOT)) {
            // TODO insert code to calculate variable shooting speed
            // SHould be based on Limlight vision calculations and distance to target
            // double[] optimalShotsResult = CalculateShooterSpeed.calculateOptimalShot(
            //     s_poseEstimate.rawFiducials[0].distToCamera, Constants.CalculateShooter.TARGET_HEIGHT);
            double[] optimalShotsResult = CalculateShooterSpeed.calculateOptimalShot(VisionSubsystem.tag_distance, 4);
            // SmartDashboard.putNumber("Optimal Shoot Values", optimalShotsResult[0]);
            
            setShooterSetpoint(optimalShotsResult[0]);
            setBackSetpoint(optimalShotsResult[1]);

            SmartDashboard.putNumber("Optimal Shooter RPS: ", optimalShotsResult[0]);
            SmartDashboard.putNumber("Optimal Back Spin RPS: ", optimalShotsResult[1]);

            // setShooterSetpoint(50);
            // setBackSetpoint(20);
            
            return;
        } else if (state.equals(ShooterStates.STOP)) {
            setShooterSetpoint(0);
            setBackSetpoint(0);
        }

        // // Otherwise use setpoint based motor speeds 
        // setBackSpinSpeed(state.backSpinRPS);
        // setAllShooterSpeed(state.shootingRPS);
    }

    public void setShooterSetpoint(double rpsGoal) {
        desiredRPS = rpsGoal; 

        if (rpsGoal != 0) {
            rightPID.setSetpoint(rpsGoal);
            middlePID.setSetpoint(rpsGoal);
            leftPID.setSetpoint(rpsGoal);
        }
    }

    public void setBackSetpoint(double rpsGoal) {
        desiredRPS = rpsGoal;

        if (rpsGoal != 0) {
            backSpinPID.setSetpoint(rpsGoal);
        }
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // TODO add data to dashboard for testing and debugging purposes
        SmartDashboard.putNumber("Right Shooter Motor Speed", m_rightShooter.get());
        SmartDashboard.putNumber("Middle Shooter Motor Speed", m_middleShooter.get());
        SmartDashboard.putNumber("Left Shooter Motor Speed", m_leftShooter.get());

        SmartDashboard.putNumber("Right Shooter Motor RPS", m_rightShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Middle Shooter Motor RPS", m_middleShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Left Shooter Motor RPS", m_leftShooter.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Right PID Calculated Val", rightPIDCalc);
        SmartDashboard.putNumber("Left PID Calculated Val", leftPIDCalc);
        SmartDashboard.putNumber("Middle PID Calculated Val", middlePIDCalc);

        SmartDashboard.putNumber("Right Back Spin Motor Speed", m_rightBackSpin.getAbsoluteEncoder().getVelocity());
        SmartDashboard.putNumber("Left Back Spin Speed", m_leftBackSpin.getAbsoluteEncoder().getVelocity()); 
        
        
        
    }
}
