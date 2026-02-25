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

public class IndexSubsystem extends SubsystemBase {
    // Intialize motors on the indexing subsystem
    // Conveyor is the front PVC that feeds into the indexor
    // Indexor feeds the balls into shooter 
    private final SparkMax m_conveyor;
    public final SparkMax m_indexing; 
    
    private IndexStates i_state; 

    public IndexSubsystem(){
        m_conveyor = new SparkMax(Constants.Indexor.kConveyorID, MotorType.kBrushless) ;
        m_indexing = new SparkMax(Constants.Indexor.kIndexorID, MotorType.kBrushless);
        
        // Optimize BUS usage 
        SparkMaxUtils.setSparkMaxBusUsage(m_conveyor, SparkMaxUtils.Usage.kMinimal, IdleMode.kCoast, false, true);
        SparkMaxUtils.setSparkMaxBusUsage(m_indexing, SparkMaxUtils.Usage.kMinimal, IdleMode.kCoast, false, false);

        // Start indexor in STOP position
        i_state = IndexStates.STOP;
        setIndexState(i_state);
    }

    @Override
    public void periodic() {
        // Update dashboard data periodically
        setDashboardData();
    }

    public void setIndexState(IndexStates state) {
        // Update index state of subsytem 
        this.i_state = state; 
        
        // Set motor speeds according to the values in Index
        m_indexing.set(state.indexerSpeed);
        // m_conveyor.set(state.conveyorSpeed);
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        
    }
}
