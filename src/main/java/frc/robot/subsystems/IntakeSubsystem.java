package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.States.IntakeStates;
import frc.lib.util.SparkFlexUtils;
import frc.lib.util.SparkMaxUtils;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
    public final SparkFlex m_intake;
    public final SparkMax m_pivotIntake; 
    private final AbsoluteEncoder pivotAbsoluteEncoder;
    private final RelativeEncoder pre; 

    private IntakeStates i_states;

    public IntakeSubsystem() {
        m_intake = new SparkFlex(Constants.Intake.kIntakeID, MotorType.kBrushless);
        m_pivotIntake = new SparkMax(Constants.Intake.kPivotID, MotorType.kBrushless);
        
        pivotAbsoluteEncoder = m_pivotIntake.getAbsoluteEncoder();
        pre = m_pivotIntake.getEncoder();
        
        // Optimize BUS usage 
        SparkFlexUtils.setSparkFlexBusUsage(m_intake, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, true);
        SparkMaxUtils.setSparkMaxBusUsage(m_pivotIntake, SparkMaxUtils.Usage.kAll, IdleMode.kBrake, false, false);

         // Start intake in STOP position
        setIntakeState(IntakeStates.STOP);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("sigma Nnija", pivotAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("sigma", pre.getPosition());
    }

    public void setIntakeState(IntakeStates state) {
        i_states = state;
        m_intake.set(state.intakeSpeed);
        m_pivotIntake.set(state.angle);
    }

    public void setPivotSpeed(double speed) {
        m_pivotIntake.set(speed);
    }

    private void setDashboardData() {
        SmartDashboard.putNumber("Intake Speed", m_intake.get());
        SmartDashboard.putNumber("Pivot Intake Speed", m_pivotIntake.get());
        SmartDashboard.putNumber("Pivot Absolute Encoder Position", pivotAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Relative Encoder Position", pre.getPosition());
    }
}
        





