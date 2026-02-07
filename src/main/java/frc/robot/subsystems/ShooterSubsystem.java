package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.States.ShooterStates;

public class ShooterSubsystem extends SubsystemBase {
    // Different motors for each channel on the robot 
    private final TalonFX m_rightShooter;
    private final TalonFX m_middleShooter;
    private final TalonFX m_leftShooter; 

    // One backspin motor for all channels 
    private final TalonFX m_backSpin;

    private ShooterStates s_state; 

    public ShooterSubsystem() {
        m_rightShooter = new TalonFX(Constants.Shooter.kShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kShootingID);

        m_backSpin = new TalonFX(Constants.Shooter.kBackspinID);

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
        m_rightShooter.set(state.shootingSpeed);
        m_middleShooter.set(state.shootingSpeed);
        m_leftShooter.set(state.shootingSpeed);

        m_backSpin.set(state.backSpinSpeed);
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // TODO add data to dashboard for testing and debugging purposes
    }
}
