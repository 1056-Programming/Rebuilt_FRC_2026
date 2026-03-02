package frc.robot.subsystems;

import java.util.zip.ZipEntry;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.lib.util.TalonFxUtils;
import frc.lib.util.Utilities;
// import frc.robot.CalculateShooterSpeed;
import frc.robot.Constants;
import frc.robot.States.ShooterStates;

import java.util.Arrays;

public class ShooterSubsystem extends SubsystemBase {
    // Different motors for each channel on the robot 
    private final TalonFX m_rightShooter;
    private final TalonFX m_middleShooter;
    private final TalonFX m_leftShooter; 

    // One backspin motor for all channels 
    private final SparkFlex m_backSpin;

    // Get backspin motor encoders for constant RPS control
    private final RelativeEncoder e_backSpin;

    // PID controllers to maintain backspin RPS
    private final SparkClosedLoopController c_backSpinPID;

    private final CommandSwerveDrivetrain drivetrain; 

    // Enable or disable subsystem
    private final boolean disable; 

    // Current shooter state
    private ShooterStates s_state;

    // Current Motor Speeds
    private double rightMotorSpeed;
    private double middleMotorSpeed;
    private double leftMotorSpeed;
    private double backSpinSpeed;

    public double desiredShooterRPS; 
    public double desiredBackSpinRPS; 

    //Boolean usePIDonShooter; // CHANGE ME
    public double[] optimalShotsResult;

    // Iterator and the HashMap for the checking the distance
    private double limelight_distance; 
    private String[] stateNames = {"DISTANCE_0_5M",
                                    "DISTANCE_1M",
                                    "DISTANCE_1_5M",
                                    "DISTANCE_2M"};
    private double[] stateDistance;

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        // Initialize Kraken Motors 
        m_rightShooter = new TalonFX(Constants.Shooter.kRightShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kMiddleShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kLeftShootingID);

        // Configure Kraken RPS PID controllers
        TalonFxUtils.configureSlot0(m_leftShooter, 0.05, 0, 0, 0, 0.13);
        TalonFxUtils.configureSlot0(m_middleShooter, 0.05, 0, 0, 0, 0.13);
        TalonFxUtils.configureSlot0(m_rightShooter, 0.05, 0, 0, 0, 0.13);

        // Initialize SparkFlex Motors
        m_backSpin = new SparkFlex(Constants.Shooter.kBackSpinID, MotorType.kBrushless);

        // Get encoders for backspin motors
        e_backSpin = m_backSpin.getEncoder();

        this.drivetrain = drivetrain;

        // Optimize CAN BUS usage
        SparkFlexUtils.setSparkFlexBusUsage(m_backSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, 
            false, true,
            0,0,0,0.000149537);
        // Initialize PID Controllers
        c_backSpinPID = m_backSpin.getClosedLoopController();

        // Initialize shooter state to STOP 
        s_state = ShooterStates.STOP;
        setShooterState(s_state);

        desiredShooterRPS = 0;
        desiredBackSpinRPS = 0; 
        // Disable Subsystem if set to true 

        disable = false; 
        if(disable) {
            disableSubsystem();
        }


    

    }
    @Override
    public void periodic() { 
        //setVelocitySetpoints(s_state.shootingRPS, s_state.backSpinRPS);
        // calculatePIDSpeed();
        
        // // If shooter setpoint is 0, set shooter motor speeds to 0 
        // // to prevent unnecessary motor wear and conserve battery life
        // if(c_rightPID.getSetpoint() == 0 || c_middlePID.getSetpoint() == 0 || c_leftPID.getSetpoint() == 0) {
        //     applyShooterMotorSpeeds(0, 0, 0);
        // } else {
        // applyShooterMotorSpeeds(rightMotorSpeed, middleMotorSpeed, leftMotorSpeed);
        // }

        // If backspin setpoint is 0, set backspin motor speeds to 0 
        // to prevent unnecessary motor wear and conserve battery life
        // if(c_backSpinPID.getSetpoint() == 0) {
        //     applyBackSpinMotorSpeeds(0);
        // } else {
        //     applyBackSpinMotorSpeeds(backSpinSpeed);
        // }

        limelight_distance = VisionSubsystem.tag_distance;
        
        setDashboardData();
    }

    public double[] checkShooterRange() {
        // double smallestDif = Math.abs(limelight_distance - stateDistance[0]);
        // double currentDif; 
        // int closest = 0; 

        // for (int i=0; i<stateDistance.length; i++) {
        //     currentDif = Math.abs(limelight_distance - stateDistance[i]);

        //     if (smallestDif < currentDif) {
        //         smallestDif = currentDif;
        //         closest = i; 
        //     }
        //  }

        //  return stateNames[closest]; 
        
        if (limelight_distance <= 0.25) {
            return new double[] {
                ShooterStates.DISTANCE_0_5M.shootingRPS,
                ShooterStates.DISTANCE_0_5M.backSpinRPS
            };
        } else if (limelight_distance <= 0.5 
                    && limelight_distance > 0.25) {
            return new double[] {
                ShooterStates.DISTANCE_1M.shootingRPS,
                ShooterStates.DISTANCE_1M.backSpinRPS
            };
        } else if (limelight_distance <= 0.75 
        && limelight_distance > 0.5) {
            return new double[] {
                ShooterStates.DISTANCE_1M.shootingRPS,
                ShooterStates.DISTANCE_1M.backSpinRPS
            };
        } else if (limelight_distance <= 1 
        && limelight_distance > 0.75) {
            return new double[] {
                ShooterStates.DISTANCE_1_5M.shootingRPS,
                ShooterStates.DISTANCE_1_5M.backSpinRPS
            };
        } else if (limelight_distance <= 1.25 
        && limelight_distance > 1) {
            return new double[] {
                ShooterStates.DISTANCE_1_5M.shootingRPS,
                ShooterStates.DISTANCE_1_5M.backSpinRPS
            };
        } else if (limelight_distance <= 1.5
        && limelight_distance > 1.25) {
            return new double[] {
                ShooterStates.DISTANCE_2M.shootingRPS,
                ShooterStates.DISTANCE_2M.backSpinRPS
            };
        } else if (limelight_distance <= 1.75
        && limelight_distance > 1.5) {
            return new double[] {
                ShooterStates.DISTANCE_2M.shootingRPS,
                ShooterStates.DISTANCE_2M.backSpinRPS
            };
        } 

        return new double[] {
            ShooterStates.FORWARD_SHOOT.shootingRPS, 
            ShooterStates.FORWARD_SHOOT.backSpinRPS
        };
    }

    public void setShooterState(ShooterStates state) {
        //stateDistance = checkShooterRange();
        s_state = state; 
        if(state.equals(ShooterStates.VARIABLE_SHOOT)) {
            // TODO:
           // setVelocitySetpoints(stateDistance[0],stateDistance[1]);
            var sigma = drivetrain.getState().Pose;
            var distance = Utilities.calculateDistanceToCenterPiece(sigma.getX(), sigma.getY());
            //var distance = limelight_distance;
            setVelocitySetpoints(Utilities.calculateShooterSpeed(distance), Utilities.calculcateBackSpinSpeed(distance));
            return;
        } 

        setVelocitySetpoints(state.shootingRPS, state.backSpinRPS);
    }

    public String getName() {
        return "Shooter Subsystem";
    }

    // 
    public void setVelocitySetpoints(double desiredShooterRPS, double desiredBackSpinRPS) {
        this.desiredShooterRPS = desiredShooterRPS; 
        this.desiredBackSpinRPS = desiredBackSpinRPS; 

        c_backSpinPID.setSetpoint(Utilities.rpsToRpm(desiredBackSpinRPS), ControlType.kVelocity);
        // Set control for desired shooter RPS
        m_leftShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
        m_middleShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
        m_rightShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
    }

    // Average RPS of both backspin motors
    private double getBackSpinRPS() {
        return e_backSpin.getVelocity() / 60;
    }

    // // Calculate PID outputs for shooter and backspin motors to maintain constant RPS
    // private void calculatePIDSpeed() {
    //     rightMotorSpeed = c_rightPID.calculate(m_rightShooter.getVelocity().getValueAsDouble());
    //     middleMotorSpeed = c_middlePID.calculate(m_middleShooter.getVelocity().getValueAsDouble());
    //     leftMotorSpeed = c_leftPID.calculate(m_leftShooter.getVelocity().getValueAsDouble()) + c_Feedforward.calculate(desiredShooterRPS); 
    //     backSpinSpeed = c_backSpinPID.calculate(getBackSpinRPS()); 
    // }

    // // Set PID setpoints for shooter and backspin motors
    // private void setPIDSetpoints(double desiredShooterRPS, double desiredBackSpinRPS) {
    //     this.desiredShooterRPS = desiredShooterRPS; 
    //     c_rightPID.setSetpoint(desiredShooterRPS);
    //     c_middlePID.setSetpoint(desiredShooterRPS);
    //     c_leftPID.setSetpoint(desiredShooterRPS);
    //     m_leftShooter.setControl(new VelocityVoltage(desiredShooterRPS));
    //     c_backSpinPID.setSetpoint(desiredBackSpinRPS);    
    // }

    // Apply motor speeds to all shooters
    // private void applyShooterMotorSpeeds(double rightSpeed, double middleSpeed, double leftSpeed) {
    //     //m_rightShooter.set(rightSpeed);
    //     //m_middleShooter.set(middleSpeed);
    //     m_leftShooter.set(leftSpeed);
    //     m_backSpin.set(backSpinSpeed);
    // }

    // Apply motor speeds to backspin motors
    // private void applyBackSpinMotorSpeeds(double backSpinSpeed) {
    //     m_backSpin.set(-backSpinSpeed);
    // }

    // Fully disable subsystem for testing purposes
    private void disableSubsystem() {
        m_rightShooter.disable();
        m_middleShooter.disable();
        m_leftShooter.disable();
        m_backSpin.disable();
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
        SmartDashboard.putNumber(getName() + " shooter PID setpoints", this.desiredShooterRPS);
        SmartDashboard.putNumber(getName() + " Back Spin PID Setpoint", this.desiredBackSpinRPS);

        // Current Shooter State
        SmartDashboard.putString(getName() + " Shooter State", s_state.toString());
                                                                                         

    }
}