package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtils;
import frc.robot.Constants;
import frc.robot.States.ShooterStates;

public class ShooterSubsystem extends SubsystemBase {
    // Different motors for each channel on the robot 
    private final TalonFX m_rightShooter;
    private final TalonFX m_middleShooter;
    private final TalonFX m_leftShooter; 

    // One backspin motor for all channels 
    private final SparkFlex m_leftBackSpin;
    private final SparkFlex m_rightBackSpin;

    private ShooterStates s_state; 

    public ShooterSubsystem() {
        m_rightShooter = new TalonFX(Constants.Shooter.kRightShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kMiddleShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kLeftShootingID);

        m_leftBackSpin = new SparkFlex(Constants.Shooter.kLeftBackspinID, MotorType.kBrushless);
        m_rightBackSpin = new SparkFlex(Constants.Shooter.kRightBackspinID, MotorType.kBrushless);

        //SparkFlexUtils.setSparkFlexBusUsage(m_leftBackSpin, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, false);
        //SparkFlexUtils.setSparkFlexBusUsage(m_rightBackSpin, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, true);

        // Initalize shooter in STOP position
        setShooterState(ShooterStates.STOP);
    }

    @Override
    public void periodic() {
        // TODO: Constantly read vision measuremnts and then calculate optimal shooting speed 
        // Only use this when target is visiable
        // Possible to use based on the odomtery reading of the robot 

        // Update dashboard data periodically
        setDashboardData();
    }

    private void setShooterState(ShooterStates state) {
        s_state = state; 

        // If using variable calculations set motor speed to optimal speed 
        if(state.equals(ShooterStates.VARIABLE_SHOOT)) {
            // TODO insert code to calculate variable shooting speed
            // SHould be based on Limlight vision calculations and distance to target
            return;
        } 

        // Otherwise use setpoint based motor speeds 
        setBackSpinSpeed(state.backSpinSpeed);
        setAllShooterSpeed(state.shootingSpeed);
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

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // TODO add data to dashboard for testing and debugging purposes
    }
}
