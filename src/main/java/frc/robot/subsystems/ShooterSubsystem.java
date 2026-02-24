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
    private final PIDController c_rightPID;
    private final PIDController c_middlePID;
    private final PIDController c_leftPID;
    private final PIDController c_backSpinPID; 

    private final boolean useRPS;

    // Current shooter state 
    private ShooterStates s_state; 

    // Motor speed 
    private double rightMotorSpeed;




    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_rightShooter = new TalonFX(Constants.Shooter.kRightShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kMiddleShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kLeftShootingID);

        m_leftBackSpin = new SparkFlex(Constants.Shooter.kLeftBackspinID, MotorType.kBrushless);
        m_rightBackSpin = new SparkFlex(Constants.Shooter.kRightBackspinID, MotorType.kBrushless);

        // Initalize PID Controllers for constant RPM wih their respective PID Constants 
        c_rightPID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kShooterD);
        c_middlePID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kShooterD);
        c_leftPID = new PIDController(Constants.Shooter.kShooterP, Constants.Shooter.kShooterI, Constants.Shooter.kShooterD);

        c_backSpinPID = new PIDController(Constants.Shooter.kBackP, Constants.Shooter.kBackI, Constants.Shooter.kBackD);

        // Optimize BUS usage
        SparkFlexUtils.setSparkFlexBusUsage(m_leftBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, false);
        SparkFlexUtils.setSparkFlexBusUsage(m_rightBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, true);

        // set usage for RPS PID 
        // Turn off for testing 
        useRPS = false; 

        // Initalize shooter in STOP position
        setShooterState(ShooterStates.STOP);
    }

    @Override
    public void periodic() {
        // TODO: Constantly read vision measuremnts and then calculate optimal shooting speed 
        // Only use this when target is visiable
        // Possible to use based on the odomtery reading of the robot 
        // NEW TODO: Have to calculate the set point for the back spin motors

        

        // Update dashboard data periodically
        setDashboardData();
    }

    private void setSetpoints(double shootSetpoint, double backSpinSetpoint) {
        c_rightPID.setSetpoint(shootSetpoint);
        c_middlePID.setSetpoint(shootSetpoint);
        c_leftPID.setSetpoint(shootSetpoint);

        c_backSpinPID.setSetpoint(backSpinSetpoint);

    }

    // create functions to set all the speed 
    private void setBackSpinSpeed(double speed) {
        m_leftBackSpin.set(speed);
        m_rightBackSpin.set(speed);
    }   

    private void setShootSpeed(double speed) {
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
            return;
        } 

        setSetpoints(state.shootingRPS, state.backSpinRPS);
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

        SmartDashboard.putNumber("Right Back Spin Motor Speed", m_rightBackSpin.get());
        SmartDashboard.putNumber("Left Back Spin Speed", m_leftBackSpin.get()); 
        
    }
}
