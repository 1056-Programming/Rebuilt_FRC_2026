package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.robot.CalculateShooterSpeed;
import frc.robot.Constants;
import frc.robot.States.ShooterStates;

public class ShooterSubsystem1 extends SubsystemBase {
    // Different motors for each channel on the robot 
    private final TalonFX m_rightShooter;
    private final TalonFX m_middleShooter;
    private final TalonFX m_leftShooter; 

    // One backspin motor for all channels 
    private final SparkFlex m_leftBackSpin;
    private final SparkFlex m_rightBackSpin;

    // Get backspin motor encoders for constant RPS control
    // private final RelativeEncoder e_leftBackSpin;
    private final RelativeEncoder e_rightBackSpin;

    // PID controllers to maintain constant RPS
    private final PIDController c_rightPID;
    private final PIDController c_middlePID;
    private final PIDController c_leftPID;
    private final PIDController c_backSpinPID;

    // Enable or disable subsystem
    private final boolean disable; 

    // Current shooter state
    private ShooterStates s_state;

    // Current Motor Speeds
    private double rightMotorSpeed;
    private double middleMotorSpeed;
    private double leftMotorSpeed;
    private double backSpinSpeed;

    public ShooterSubsystem1() {
        // Initialize Kraken Motors 
        m_rightShooter = new TalonFX(Constants.Shooter.kRightShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kMiddleShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kLeftShootingID);

        // Initialize SparkFlex Motors
        m_leftBackSpin = new SparkFlex(Constants.Shooter.kLeftBackspinID, MotorType.kBrushless);
        m_rightBackSpin = new SparkFlex(Constants.Shooter.kRightBackspinID, MotorType.kBrushless);

        // Get encoders for backspin motors
        //e_leftBackSpin = m_leftBackSpin.getEncoder();
        e_rightBackSpin = m_rightBackSpin.getEncoder();

        // Optimize CAN BUS usage
        SparkFlexUtils.setSparkFlexBusUsage(m_leftBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, true);
        SparkFlexUtils.setSparkFlexBusUsage(m_rightBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, false);

        // Initialize PID Controllers
        c_rightPID = new PIDController(Constants.Shooter.ShooterKP, Constants.Shooter.ShooterKI, Constants.Shooter.ShooterKD);
        c_middlePID = new PIDController(Constants.Shooter.ShooterKP, Constants.Shooter.ShooterKI, Constants.Shooter.ShooterKD);
        c_leftPID = new PIDController(Constants.Shooter.ShooterKP, Constants.Shooter.ShooterKI, Constants.Shooter.ShooterKD);
        c_backSpinPID = new PIDController(Constants.Shooter.BackSpinKP, Constants.Shooter.BackSpinKI, Constants.Shooter.BackSpinKD); 
        
        // Set PID Tolerance
        c_rightPID.setTolerance(0);
        c_middlePID.setTolerance(0);
        c_leftPID.setTolerance(0);
        c_backSpinPID.setTolerance(0.05);

        // Initialize shooter state to STOP 
        s_state = ShooterStates.STOP;
        setShooterState(s_state);

        // Disable Subsystem if set to true 
        disable = false; 
        if(disable) {
            disableSubsystem();
        }
    }

    @Override
    public void periodic() { 
        calculatePIDSpeed();
        
        // If shooter setpoint is 0, set shooter motor speeds to 0 
        // to prevent unnecessary motor wear and conserve battery life
        if(c_rightPID.getSetpoint() == 0 || c_middlePID.getSetpoint() == 0 || c_leftPID.getSetpoint() == 0) {
            applyShooterMotorSpeeds(0, 0, 0);
        } else {
            applyShooterMotorSpeeds(rightMotorSpeed, middleMotorSpeed, leftMotorSpeed);
        }

        // If backspin setpoint is 0, set backspin motor speeds to 0 
        // to prevent unnecessary motor wear and conserve battery life
        if(c_backSpinPID.getSetpoint() == 0) {
            applyBackSpinMotorSpeeds(0);
        } else {
            applyBackSpinMotorSpeeds(backSpinSpeed);
        }

        setDashboardData();
    }
    
    public void setShooterState(ShooterStates state) {
        s_state = state; 
        if(state.equals(ShooterStates.VARIABLE_SHOOT)) {
            // TODO:
            double[] optimalShotsResult = CalculateShooterSpeed.calculateOptimalShot(VisionSubsystem.tag_distance, 4);
            // SmartDashboard.putNumber("Optimal Shoot Values", optimalShotsResult[0]);
            
            setPIDSetpoints(optimalShotsResult[0], 40);

            SmartDashboard.putNumber("Optimal Shooter RPS: ", optimalShotsResult[0]);
            SmartDashboard.putNumber("Optimal Back Spin RPS: ", -optimalShotsResult[1]);

            return;
        } 

        setPIDSetpoints(state.shootingRPS, state.backSpinRPS);
    }

    public String getName() {
        return "Shooter Subsystem";
    }

    // Calculate PID outputs for shooter and backspin motors to maintain constant RPS
    private void calculatePIDSpeed() {
        rightMotorSpeed = c_rightPID.calculate(m_rightShooter.getVelocity().getValueAsDouble());
        middleMotorSpeed = c_middlePID.calculate(m_middleShooter.getVelocity().getValueAsDouble());
        leftMotorSpeed = c_leftPID.calculate(m_leftShooter.getVelocity().getValueAsDouble());
        backSpinSpeed = -c_backSpinPID.calculate(getBackSpinRPS());
    }

    // Average RPS of both backspin motors
    private double getBackSpinRPS() {
       // return (e_leftBackSpin.getVelocity() + e_rightBackSpin.getVelocity()) / 2.0; 
       return e_rightBackSpin.getVelocity() / 60 ;
    }

    // Set PID setpoints for shooter and backspin motors
    private void setPIDSetpoints(double desiredShooterRPS, double desiredBackSpinRPS) {
        c_rightPID.setSetpoint(desiredShooterRPS);
        c_middlePID.setSetpoint(desiredShooterRPS);
        c_leftPID.setSetpoint(desiredShooterRPS);
        c_backSpinPID.setSetpoint(desiredBackSpinRPS);    
    }

    // Apply motor speeds to all shooters
    private void applyShooterMotorSpeeds(double rightSpeed, double middleSpeed, double leftSpeed) {
        //m_rightShooter.set(rightSpeed);
        //m_middleShooter.set(middleSpeed);
        m_leftShooter.set(leftSpeed);
        m_leftBackSpin.set(backSpinSpeed);
        m_rightBackSpin.set(backSpinSpeed);
    }

    // Apply motor speeds to backspin motors
    private void applyBackSpinMotorSpeeds(double backSpinSpeed) {
        // m_leftBackSpin.set(backSpinSpeed);
        m_rightBackSpin.set(backSpinSpeed);
    }

    // Fully disable subsystem for testing purposes
    private void disableSubsystem() {
        m_rightShooter.disable();
        m_middleShooter.disable();
        m_leftShooter.disable();
        m_leftBackSpin.disable();
        m_rightBackSpin.disable();
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // Motor speeds
        SmartDashboard.putNumber(getName() + " Right Shooter Motor Speed", rightMotorSpeed);
        SmartDashboard.putNumber(getName() + " Middle Shooter Motor Speed", middleMotorSpeed);
        SmartDashboard.putNumber(getName() + " Left Shooter Motor Speed", leftMotorSpeed);
        SmartDashboard.putNumber(getName() + " Back Spin Motor Speed", backSpinSpeed);

        // Motor RPS
        SmartDashboard.putNumber(getName() + " Right Shooter RPS", m_rightShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(getName() + " Middle Shooter RPS", m_middleShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(getName() + " Left Shooter RPS", m_leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(getName() + " Back Spin RPS", getBackSpinRPS());

        // PID Setpoint Values
        SmartDashboard.putNumber(getName() + " Right Shooter PID Setpoint", c_rightPID.getSetpoint());
        SmartDashboard.putNumber(getName() + " Middle Shooter PID Setpoint", c_middlePID.getSetpoint());
        SmartDashboard.putNumber(getName() + " Left Shooter PID Setpoint", c_leftPID.getSetpoint());
        SmartDashboard.putNumber(getName() + " Back Spin PID Setpoint", c_backSpinPID.getSetpoint());

        // Current Shooter State
        SmartDashboard.putString(getName() + " Shooter State", s_state.toString());
    }
}