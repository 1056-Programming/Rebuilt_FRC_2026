package frc.robot.subsystems;

import org.opencv.video.DenseOpticalFlow;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.robot.CalculateShooterSpeed;
import frc.robot.Constants;
import frc.robot.States.ShooterStates;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.VisionSubsystem; 

public class ShooterSubsystem extends SubsystemBase {
    // Different motors for each channel on the robot 
    private final TalonFX m_rightShooter;
    private final TalonFX m_middleShooter;
    private final TalonFX m_leftShooter; 

    // One backspin motor for all channels 
    private final SparkFlex m_leftBackSpin;
    private final SparkFlex m_rightBackSpin;

<<<<<<< HEAD
    // Current shooter state 
    private ShooterStates s_state; 

    public ShooterSubsystem() {
=======
    // Shooter PID to maintain cconstants RPS 
    private final PIDController rightPID;
    private final PIDController middlePID;
    private final PIDController leftPID;

    // Backspin PID to maintain cconstants RPS 
    private final PIDController backSpinPID;

    // Current shooter state 
    private ShooterStates s_state; 

    private VisionSubsystem s_limeDist; 

    private PoseEstimate s_poseEstimate; 

    private double rightPIDCalc;
    private double middlePIDCalc;
    private double leftPIDCalc; 

    private double DesiredRPS; 

    public double[] optimalShotsResult; 


    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
>>>>>>> 247593aab8e295f17e7173707608dbf4c1e94fd8
        m_rightShooter = new TalonFX(Constants.Shooter.kRightShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kMiddleShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kLeftShootingID);

        m_leftBackSpin = new SparkFlex(Constants.Shooter.kLeftBackspinID, MotorType.kBrushless);
        m_rightBackSpin = new SparkFlex(Constants.Shooter.kRightBackspinID, MotorType.kBrushless);

<<<<<<< HEAD
=======
        backSpinPID = new PIDController(Constants.Shooter.kBackP, Constants.Shooter.kBackI, Constants.Shooter.kBackD); 

        s_poseEstimate = new PoseEstimate(); 

        s_limeDist = new VisionSubsystem(drivetrain); // unsure on where and how to get the drivetrain

>>>>>>> 247593aab8e295f17e7173707608dbf4c1e94fd8
        SparkFlexUtils.setSparkFlexBusUsage(m_leftBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, false);
        SparkFlexUtils.setSparkFlexBusUsage(m_rightBackSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, false, true);

        setShooterSetpoint(0);
        setBackSetpoint(0);

        optimalShotsResult = new double[2];

        // Initalize shooter in STOP position
        setShooterState(ShooterStates.STOP);
        LimelightHelpers.setPipelineIndex("limelight-hotrock", 0);

    }

    @Override
    public void periodic() {
        // TODO: Constantly read vision measuremnts and then calculate optimal shooting speed 
        // Only use this when target is visiable
        // Possible to use based on the odomtery reading of the robot 
<<<<<<< HEAD

=======
        // NEW TODO: Have to calculate the set point for the back spin motors
        
        if (DesiredRPS == 0) {
            setAllShooterSpeed(0);
        } else {
            m_rightShooter.set(
                rightPIDCalc = rightPID.calculate(m_rightShooter.getVelocity().getValueAsDouble())
            );
            
            m_leftShooter.set(
                leftPIDCalc = leftPID.calculate(m_leftShooter.getVelocity().getValueAsDouble())
            );

<<<<<<< HEAD
            m_middleShooter.set(
                middlePIDCalc = middlePID.calculate(m_middleShooter.getVelocity().getValueAsDouble())
            );

            m_leftBackSpin.set(
                backSpinPID.calculate(m_leftBackSpin.get())
            );

            m_rightBackSpin.set(
                backSpinPID.calculate(m_rightBackSpin.get())
            );
        }
        
=======
        m_middleShooter.set(
            middlePIDCalc = middlePID.calculate(m_middleShooter.getVelocity().getValueAsDouble())
        );
>>>>>>> 247593aab8e295f17e7173707608dbf4c1e94fd8
>>>>>>> f66c78ef3489385d36716339c622ba51a1417896

        // Update dashboard data periodically
        setDashboardData();
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

<<<<<<< HEAD
    private void setShooterState(ShooterStates state) {
=======
    public void setShooterState(ShooterStates state) {
>>>>>>> 247593aab8e295f17e7173707608dbf4c1e94fd8
        s_state = state; 

        // If using variable calculations set motor speed to optimal speed 
        if(state.equals(ShooterStates.VARIABLE_SHOOT)) {
            // TODO insert code to calculate variable shooting speed
            // SHould be based on Limlight vision calculations and distance to target
            optimalShotsResult = CalculateShooterSpeed.calculateOptimalShot(
                s_poseEstimate.rawFiducials[0].distToCamera, Constants.CalculateShooter.TARGET_HEIGHT);
            
            setShooterSetpoint(optimalShotsResult[0]);
            setBackSetpoint(optimalShotsResult[1]);
            
            return;
        } else {
            setShooterSetpoint(0);
            setBackSetpoint(0);
        }

        // // Otherwise use setpoint based motor speeds 
        // setBackSpinSpeed(state.backSpinRPS);
        // setAllShooterSpeed(state.shootingRPS);
    }

    public void setShooterSetpoint(double desiredRPS) {
        DesiredRPS = desiredRPS; 

        if (desiredRPS == 0) {
            setAllShooterSpeed(0);
        } else {
            rightPID.setSetpoint(desiredRPS);
            middlePID.setSetpoint(desiredRPS);
            leftPID.setSetpoint(desiredRPS);
        }
    }

    public void setBackSetpoint(double desiredRPS) {
        DesiredRPS = desiredRPS;
        if (desiredRPS == 0) {
            setBackSpinSpeed(0); 
        } else {
            backSpinPID.setSetpoint(desiredRPS);
        }
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // TODO add data to dashboard for testing and debugging purposes
        SmartDashboard.putNumber("Right Shooter Motor Speed", m_rightShooter.get());
        SmartDashboard.putNumber("Middle Shooter Motor Speed", m_middleShooter.get());
        SmartDashboard.putNumber("Left Shooter Motor Speed", m_leftShooter.get());

        SmartDashboard.putNumber("Right Shooter Motor RPS", m_rightShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Middle Shooter Motor RPS", m_middleShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Left Shooter Motor RPS", m_leftShooter.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Right PID Calculated Val", rightPIDCalc);
        SmartDashboard.putNumber("Left PID Calculated Val", leftPIDCalc);
        SmartDashboard.putNumber("Middle PID Calculated Val", middlePIDCalc);

        SmartDashboard.putNumber("Right Back Spin Motor Speed", m_rightBackSpin.get());
        SmartDashboard.putNumber("Left Back Spin Speed", m_leftBackSpin.get()); 
        
    }
}
