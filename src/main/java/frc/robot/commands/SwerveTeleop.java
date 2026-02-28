package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.LRUMap;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.lib.util.Utilities;

// Ensure smooth acceleration with rapid decleration 
public class SwerveTeleop extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;

    // Set max speeds for swerve driving
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double deadband = 0.1;

    // PID for auto tag yawing
    private final PIDController c_yawPID; 

    // Setting up bindings for necessary control of the swerve drive platform 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband) // Add a 10% deadband
            .withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private double xInput, yInput, rInput; 
    private double xSpeed, ySpeed, rSpeed;

    public SwerveTeleop(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        // Initialize drivetrain and controller
        this.drivetrain = drivetrain; 
        this.controller = controller;

        // Intialize controller inputs to 0
        xInput = 0; 
        yInput = 0;
        rInput = 0; 

        // Intiatlize swerve speeds to 0 
        xSpeed = 0;
        ySpeed = 0;
        rSpeed = 0;

        c_yawPID = new PIDController(0,0,0);

        // Set requiremnts for the drivetrain subsystem to ensure no conflicts with other commands
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Set contoller speeds 
        xInput = controller.getLeftY();
        yInput = controller.getLeftX();
        rInput = -controller.getRightX();

        setPolynomialAcceleration();

        drivetrain.applyRequest(() -> drive.withVelocityX(ySpeed)
            .withVelocityY(xSpeed)
            .withRotationalRate(rSpeed)).execute();
        SmartDashboard.putNumber("ten big sigmas in abar", Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
        SmartDashboard.putNumber("9 67", drivetrain.getState().Pose.getRotation().getDegrees());
            SmartDashboard.putNumber("sigma buttt nga x",drivetrain.getState().Pose.getMeasureX().baseUnitMagnitude());
            SmartDashboard.putNumber("sigma buttt nga y",drivetrain.getState().Pose.getMeasureY().baseUnitMagnitude());
            SmartDashboard.putNumber("sigma buttt nga z",drivetrain.getState().Pose.getRotation().getDegrees());
    }
    
    // Apply a polynomial acceleration curve to the joystick inputs for smoother control
    private void setPolynomialAcceleration() {
        xSpeed = Utilities.polynomialAccleration(yInput) * MaxSpeed * 0;
        ySpeed = Utilities.polynomialAccleration(xInput) * MaxSpeed * 0.2;
        rSpeed = Utilities.polynomialAccleration(rInput) * MaxAngularRate * 0.7 ; 
    }

    private void autoYaw(boolean Enable) {
        c_yawPID.setSetpoint(0);
    }

    private void setDashboardData() {
        // TODO add data to dashboard for testing and debugging purposes
    }
}