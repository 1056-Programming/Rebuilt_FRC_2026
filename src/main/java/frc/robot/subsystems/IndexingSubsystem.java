package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtils;
import frc.lib.util.SparkMaxUtils;

import frc.robot.Constants;
import frc.robot.States.IndexStates;

public class IndexingSubsystem extends SubsystemBase{
    // Intialize motors on the indexing subsystem
    // Conveyor is the front PVC that feeds into the indexor
    // Indexor feeds the balls into shooter 
    private final SparkFlex m_conveyor;
    private final SparkMax m_Indexing; 
    
    private IndexStates i_state; 

    public IndexingSubsystem(){
        m_conveyor = new SparkFlex(Constants.Indexing.kConveyorID, MotorType.kBrushless) ;
        m_Indexing = new SparkMax(Constants.Indexing.kIndexerID, MotorType.kBrushless);

        // Optimize BUS usage 
        SparkFlexUtils.setSparkFlexBusUsage(m_conveyor, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, false);
        SparkMaxUtils.setSparkMaxBusUsage(m_Indexing, SparkMaxUtils.Usage.kMinimal, IdleMode.kCoast, false, false);

        // Start indexor in STOP position
        setIndexState(IndexStates.STOP);
    }

    @Override
    public void periodic() {
        // Update dashboard data periodically
        setDashboardData();
    }

    private void setIndexState(IndexStates state) {
        // Update index state of subsytem 
        i_state = state; 
        
        // Set motor speeds according to the values in Index
        m_Indexing.set(state.indexerSpeed);
        m_conveyor.set(state.conveyorSpeed);
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        
    }
}
