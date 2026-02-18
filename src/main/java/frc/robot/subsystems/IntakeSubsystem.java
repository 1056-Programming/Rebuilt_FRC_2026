package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.States.IntakeStates;
import frc.lib.util.SparkFlexUtils;
import frc.lib.util.SparkMaxUtils;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
    public final SparkFlex m_intake;
    public final SparkMax m_pivotIntake; 
    private final AbsoluteEncoder pivotAbsoluteEncoder;

    private IntakeStates i_states;

    public IntakeSubsystem(){
        m_intake = new SparkFlex(Constants.Intake.kIntakeID, MotorType.kBrushless);
        m_pivotIntake = new SparkMax(Constants.Intake.kPivotID, MotorType.kBrushless);
        
        pivotAbsoluteEncoder = m_pivotIntake.getAbsoluteEncoder();
        
        // Optimize BUS usage 
        SparkFlexUtils.setSparkFlexBusUsage(m_intake, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, true);
        SparkMaxUtils.setSparkMaxBusUsage(m_pivotIntake, SparkMaxUtils.Usage.kMinimal, IdleMode.kBrake, false, false);
         // Start intake in STOP position
        setIntakeState(IntakeStates.STOP);

     }
    @Override
    public void periodic() {

    }

    public void setIntakeState(IntakeStates state) {
        i_states = state;
        m_intake.set(state.intakeSpeed);
        m_pivotIntake.set(state.angle);
    }

    public void setPivotSpeed(double speed) {
        m_pivotIntake.set(speed);
    }
}
        





