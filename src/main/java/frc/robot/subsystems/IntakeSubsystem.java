package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.lib.util.SparkMaxUtils;
import frc.robot.Constants;
import frc.robot.States.IntakeStates;

public class IntakeSubsystem extends SubsystemBase {
    // Spark Flex controlling ball intake
    // Spark Max controlling pivot of intake
    private final SparkFlex m_intake;
    public final SparkMax m_pivot; 

    // Absolute Encorder to track pivot angle
    private final CANcoder pivotEncoder;

    // PID controller to maintain pivot angle
    private final PIDController c_pivotPID;
    private final ArmFeedforward c_ArmFeedforward; 
    
    // Current intake state
    private IntakeStates i_state;

    // Calculated PID Speed for pivot
    private double pivotSpeed;

    // Enable or disable subsystem
    private final boolean disable;

    public IntakeSubsystem() {
        // Initialize Spark Flex, Spark Max motors, and Throguh Bore Cancoder
        m_intake = new SparkFlex(Constants.Intake.kIntakeID, MotorType.kBrushless);
        m_pivot = new SparkMax(Constants.Intake.kPivotID, MotorType.kBrushless);
        pivotEncoder = new CANcoder(Constants.Intake.kEncoderID);

        // Optimize BUS usage
        SparkFlexUtils.setSparkFlexBusUsage(m_intake, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, true);
        SparkMaxUtils.setSparkMaxBusUsage(m_pivot, SparkMaxUtils.Usage.kAll, IdleMode.kBrake, false, false);

        // Initialize PID controller for pivot
        c_pivotPID = new PIDController(Constants.Intake.kIntakeP, Constants.Intake.kIntakeI, Constants.Intake.kIntakeD);
        c_ArmFeedforward = new ArmFeedforward(0, 0.015, 0.33);

        // Start intake in STOP position
        i_state = IntakeStates.STOP;
        setIntakeState(i_state);

        // Disable Subsystem if set to true 
        disable = true;
        if(disable) {
            disableSubsystem();
        }
    }

    @Override
    public void periodic() {
        // Set motor speed based on PID calculation
        // pivotSpeed = c_pivotPID.calculate(getPiviotPosition()) 
        //     + c_ArmFeedforward.calculate(Units.degreesToRadians(getPiviotPosition()), pivotEncoder.getVelocity().getValueAsDouble());
        // if(pivotSpeed < 0) {
        //     pivotSpeed *= 0.8;
        // }
        // m_pivot.set(pivotSpeed);
        setDashboardData();
    }


    // Adjust Subsytem to desired Intake states 
    public void setIntakeState(IntakeStates state) {
        i_state = state;
        
        // Set PID setpoint on intake to calculate motor output during periodic
        c_pivotPID.setSetpoint(state.pivotAngle);
        m_intake.set(state.intakeSpeed);
    }

    public String getName() {
        return "Intake Subsystem";
    }

    // Return position of encoder in degrees
    // For some reasons it negative
    // Increases as you go up 
    private double getPiviotPosition() {
        return Units.rotationsToDegrees(pivotEncoder.getPosition().getValueAsDouble()) * -1 -60 * 4;
    }

    // Disable susbystem if needed
    private void disableSubsystem() {
        m_intake.disable();
        m_pivot.disable();
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // Put current state on the dashboard
        SmartDashboard.putString(getName() + " current state", i_state.toString());

        // Put motor speeds and pid setpoints
        SmartDashboard.putNumber(getName() + " pivot setpoint", c_pivotPID.getSetpoint());
        SmartDashboard.putNumber(getName() + " pivot speed", pivotSpeed);
        SmartDashboard.putNumber(getName() + " intake speed", m_intake.get());
        SmartDashboard.putNumber(getName() + " piviot position", getPiviotPosition());
    }
}
