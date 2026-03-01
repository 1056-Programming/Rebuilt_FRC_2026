package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.events.CancelCommandEvent;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.lib.util.SparkMaxUtils;
import frc.lib.util.Utilities;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.Constants.Intake;
import frc.robot.States.IntakeStates;

public class IntakeSubsystem extends SubsystemBase {
    // Spark Flex controlling ball intake
    private final SparkFlex m_intake;

    // Spark Max controlling pivot of intake
    public final SparkMax m_pivot; 

    // Absolute Encorder to track pivot angle
    private final CANcoder pivotEncoder;

    // PID controller to maintain pivot angle
    private final PIDController c_pivotPID;

    // Enable or disable subsystem
    private final boolean disable;
    
    // Current intake state
    private IntakeStates i_state;

    private double motorSpeed = 0; 

    private double setpoint; 

    // Calculated PID Speed for pivot
    private double pivotSpeed;

    public IntakeSubsystem() {
        // Initialize Spark Flex and Spark Max motors
        m_intake = new SparkFlex(Constants.Intake.kIntakeID, MotorType.kBrushless);
        m_pivot = new SparkMax(31, MotorType.kBrushless);
        pivotEncoder = new CANcoder(23);
        CANcoderConfiguration sigma = new CANcoderConfiguration();



        // Optimize BUS usage
        SparkFlexUtils.setSparkFlexBusUsage(m_intake, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, true);
        SparkMaxUtils.setSparkMaxBusUsage(m_pivot, SparkMaxUtils.Usage.kAll, IdleMode.kBrake, false, false);
              //  0, 0, 0, 0.333, pivotEncoder);

        // Initialize PID controller for pivot
        c_pivotPID = new PIDController(0.078,0,0);
        c_pivotPID.setSetpoint(-150);

        // Start intake in STOP position
        i_state = IntakeStates.STOP;
        setIntakeState(i_state);

        c_pivotPID.setSetpoint(-150);

        // Disable Subsystem if set to true 
        disable = false;
        if(disable) {
            disableSubsystem();
        }
    }

    @Override
    public void periodic() {
        // TODO INSERT ENCODER VALUE LATER 
        // pivotSpeed = c_pivotPID.calculate(0);
        //m_pivot.set(0.1);
        
        // implement this when desired angle is found 
        // m_pivot.set(
        //     c_pivotPID.calculate(Units.m_pivot.getAbsoluteEncoder().getPosition()));
        // m_pivot.set(c_pivotPID.calculate(
        //     Units.rotationsToDegrees(pivotEncoder.getPosition().getValueAsDouble())));
        m_pivot.set(motorSpeed);


        SmartDashboard.putNumber("siga", c_pivotPID.calculate(
            Units.rotationsToDegrees(pivotEncoder.getPosition().getValueAsDouble())));

        setDashboardData();

    }

    public void setSetpoint(double setpoint) {
        c_pivotPID.setSetpoint(setpoint);
    }

    public void setIntakeState(IntakeStates state) {
        i_state = state;
        
        // Set PID setpoint on intake to calculate motor output during periodic
        c_pivotPID.setSetpoint(state.pivotAngle);
        m_intake.set(state.intakeSpeed);
    }

    public void moveIntake(double speed) {
        motorSpeed = speed;
    }

    public String getName() {
        return "Intake Subsystem";
    }

    // Disable susbystem if needed
    private void disableSubsystem() {
        m_intake.disable();
        m_pivot.disable();
    }

    private void setDashboardData() {
        SmartDashboard.putNumber(getName() + " Motor Speed", pivotSpeed);
        SmartDashboard.putNumber(getName() + " Pivot Angle", Units.rotationsToDegrees(pivotEncoder.getAbsolutePosition().getValueAsDouble()));
        SmartDashboard.putNumber(getName() + " Pivot Setpoint", c_pivotPID.getSetpoint());

        SmartDashboard.putNumber(getName() + " Intake Speed", m_intake.get());

        SmartDashboard.putString(getName() + " State", i_state.toString());
    }
}
// //