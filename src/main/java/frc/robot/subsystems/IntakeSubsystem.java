package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.lib.util.SparkMaxUtils;

import frc.robot.Constants;
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

    // Calculated PID Speed for pivot
    private double pivotSpeed;

    public IntakeSubsystem() {
        // Initialize Spark Flex and Spark Max motors
        m_intake = new SparkFlex(Constants.Intake.kIntakeID, MotorType.kBrushless);
        m_pivot = new SparkMax(31, MotorType.kBrushless);

        // Optimize BUS usage
        SparkFlexUtils.setSparkFlexBusUsage(m_intake, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, true);
        SparkMaxUtils.setSparkMaxBusUsage(m_pivot, SparkMaxUtils.Usage.kAll, IdleMode.kBrake, false, false);

        pivotEncoder = new CANcoder(23);

        // Initialize PID controller for pivot
        c_pivotPID = new PIDController(Constants.Intake.kIntakeP, Constants.Intake.kIntakeI, Constants.Intake.kIntakeD);

        // Start intake in STOP position
        i_state = IntakeStates.STOP;
        setIntakeState(i_state);

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
        //c_pivotPID.calculate(m_pivot.getAbsoluteEncoder().getPosition());
        m_pivot.set(0.1);

        setDashboardData();
    }

    public void setIntakeState(IntakeStates state) {
        i_state = state;
        
        c_pivotPID.setSetpoint(state.pivotAngle);
        m_intake.set(state.intakeSpeed);
    }

    public void moveIntake(double speed) {
        motorSpeed = speed;
    }

    public String getName() {
        return "Intake Subsystem";
    }

    private void disableSubsystem() {
        m_intake.disable();
        m_pivot.disable();
    }

    private void setDashboardData() {
        SmartDashboard.putNumber(getName() + " Motor Speed", pivotSpeed);
        SmartDashboard.putNumber(getName() + " Pivot Angle", 0);
        SmartDashboard.putNumber(getName() + " Pivot Setpoint", c_pivotPID.getSetpoint());

        SmartDashboard.putNumber(getName() + " Intake Piviot Angle", Units.rotationsToDegrees(m_intake.getEncoder().getPosition()));

        SmartDashboard.putNumber(getName() + " Intake Speed", m_intake.get());

        SmartDashboard.putString(getName() + " State", i_state.toString());
        SmartDashboard.putNumber("sigma boy", Units.rotationsToDegrees(pivotEncoder.getPosition().getValueAsDouble()));
    }
}
