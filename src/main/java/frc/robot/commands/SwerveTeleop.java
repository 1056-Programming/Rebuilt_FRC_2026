package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.lib.util.Utilities;


// Ensure smooth acceleration with rapid decleration 
public class SwerveTeleop extends Command {
    private final CommandSwerveDrivetrain drivetrain; 
    private final CommandXboxController controller;

    // Set max speeds for swerve driving
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double deadband = 0.1;

    // Setting up bindings for necessary control of the swerve drive platform 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband) // Add a 10% deadband
            .withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private double xSpeed, ySpeed;
    private double prevXInput, prevYInput; 

    public SwerveTeleop(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain; 
        this.controller = controller;

        xSpeed = 0; 
        ySpeed = 0; 
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
    //     ySpeed = calculateTranslation(controller.getLeftX());
    //     xSpeed = calculateTranslation(controller.getLeftY());

        // simple polynimal acceleration curve    
        drivetrain.applyRequest(() -> drive.withVelocityX(Utilities.polynomialAccleration(-xSpeed * MaxSpeed))
            .withVelocityY(Utilities.polynomialAccleration(-ySpeed * MaxSpeed))
            .withRotationalRate(-controller.getRightX() * MaxAngularRate)).execute();
    }

    // private double calculateTranslation(double input) {
    //     if(input > 0.05) {
    //         return 0.05;
    //     } else if (input < -0.05) {
    //         return -0.05;
    //     } else {
    //         return input;
    //     }
    // }
}