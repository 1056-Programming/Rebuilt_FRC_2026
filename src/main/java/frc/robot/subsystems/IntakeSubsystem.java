package frc.robot.subsystems;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.States.IntakeStates;
import frc.lib.util.SparkFlexUtils;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex m_right;
    private final SparkFlex m_left;

    private IntakeStates i_states;

    public IntakeSubsystem(){
        m_right = new SparkFlex(Constants.Intake.kRight, MotorType.kBrushless);
        m_left = new SparkFlex(Constants.Intake.kLeft, MotorType.kBrushless);

        // Optimize BUS usage 
        SparkFlexUtils.setSparkFlexBusUsage(m_right, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, false);
        SparkFlexUtils.setSparkFlexBusUsage(m_left, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, false);
         // Start intake in STOP position
        setIntakeState(IntakeStates.STOP);

     }
    @Override
    public void periodic() {
        





