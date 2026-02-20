package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.LRUMap;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
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

    // Set slew limiters for Translation
    private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(Constants.kTranslationLimiter);
    private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(Constants.kTranslationLimiter);

    // Setting up bindings for necessary control of the swerve drive platform 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband) // Add a 10% deadband
            .withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private double xInput, yInput, rInput; 
    private double xSpeed, ySpeed, rSpeed;
    private double prevXInput, prevYInput, prevRInput;

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

        // Initalize previous Inputs to 0
        prevXInput = 0;
        prevYInput = 0;
        prevRInput = 0;

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
        //setSlewAcceleration();

        drivetrain.applyRequest(() -> drive.withVelocityX(ySpeed)
            .withVelocityY(xSpeed)
            .withRotationalRate(rSpeed)).execute();
        
        // Set previous inputs for SlewAcceleration
        prevXInput = xInput;
        prevYInput = yInput;
        prevRInput = rInput;
    }
    
    // Apply a polynomial acceleration curve to the joystick inputs for smoother control
    private void setPolynomialAcceleration() {
        xSpeed = Utilities.polynomialAccleration(yInput) * MaxSpeed;
        ySpeed = Utilities.polynomialAccleration(xInput) * MaxSpeed;
        rSpeed = Utilities.polynomialAccleration(rInput) * MaxAngularRate; 
    }

    // Incermentally increase speed with a slew rate limiter, 
    // but allow for quick decleration by not limiting negative changes in input
    private void setSlewAcceleration() {
        if(Math.abs(xInput) < Math.abs(prevXInput)) {
            xSpeed = prevXInput;
        } else {
            xSpeed = xSlewLimiter.calculate(xInput);
        }

        if(Math.abs(yInput) < Math.abs(prevYInput)) {
            ySpeed = prevYInput;
        } else {
            ySpeed = ySlewLimiter.calculate(yInput);
        }

        rSpeed = rInput * MaxAngularRate; // No slew rate limiter for rotation to allow for quick turns
    }

    private void setDashboardData() {
        // TODO add data to dashboard for testing and debugging purposes
    }
}