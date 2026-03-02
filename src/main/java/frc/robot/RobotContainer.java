// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.equation.VariableType;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.Intake;

import frc.robot.States.IndexStates;
import frc.robot.States.IntakeStates;
import frc.robot.States.ShooterStates;

import frc.robot.commands.IndexCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.IntakeCommand;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);

    //** Initialize Subsystems **//
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ShooterSubsystem s_shooter = new ShooterSubsystem();
    private final IndexSubsystem s_indexor = new IndexSubsystem();
    private final IntakeSubsystem s_intake = new IntakeSubsystem();
    private final VisionSubsystem s_VisionSubsystem = new VisionSubsystem(drivetrain,"limelight-hotrock");

    //** Initialize Commands **//
    private final ShooterCommand c_shooterCommand = new ShooterCommand(s_shooter);
    private final IndexCommand c_indexCommand = new IndexCommand(s_indexor);
    private final IntakeCommand c_intakeCommand = new IntakeCommand(s_intake);
    private final SwerveTeleop c_teleop = new SwerveTeleop(drivetrain, driver0);   
    private SendableChooser<Command> m_chooser;


    public RobotContainer() {
        drivetrain.resetPose(new Pose2d(3.7, 4.2, new Rotation2d()));
        configureBindings();
        setDriverBindings();
        configureAuto();
    }

    private void configureBindings() {     
        // Bind Shooter Commands 
        // driver1.pov(0).onTrue(c_shooterCommand.setShooterState(States.ShooterStates.FORWARD_SHOOT));
        // driver1.pov(90).onTrue(c_shooterCommand.setShooterState(States.ShooterStates.MAX));
        // driver1.pov(180).onTrue(c_shooterCommand.setShooterState(States.ShooterStates.TEST1));
        // driver1.pov(270).onTrue(c_shooterCommand.setShooterState(States.ShooterStates.VARIABLE_SHOOT));
        // driver1.pov(-1).onTrue(c_shooterCommand.setShooterState(States.ShooterStates.STOP));

        // Bind shooter controls
        driver1.y().onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS+=1, s_shooter.desiredBackSpinRPS)));
        driver1.y().onFalse(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));

        driver1.a().onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS+=0.2)));
        driver1.a().onFalse(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));
        
        driver1.rightTrigger().onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS-=1, s_shooter.desiredBackSpinRPS)));
        driver1.rightTrigger().onFalse(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));

        driver1.x().onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS-=0.2)));
        driver1.x().onFalse(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));


        // driver1.rightTrigger().onTrue(c_shooterCommand.setShooterState(States.ShooterStates.VARIABLE_SHOOT));
        // driver1.rightTrigger().onFalse(c_shooterCommand.setShooterState(States.ShooterStates.STOP)); 
        
        driver1.pov(90).onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS*0, s_shooter.desiredBackSpinRPS*0)));
        driver1.pov(90).onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));


        // Bind Intake Piviot Controls (for determining optimal angle)
        // driver1.x().whileTrue(new InstantCommand(() -> s_intake.moveIntake(0.4)));
        // driver1.x().onFalse(new InstantCommand(() -> s_intake.moveIntake(0)));

        // driver1.b().whileTrue(new InstantCommand(() -> s_intake.moveIntake(-0.2)));
        // driver1.b().onFalse(new InstantCommand(() -> s_intake.moveIntake(0)));     
        
        // Bind Intake Piviot Controls (testing and setting the PIDs)
        
        // driver1.x().onTrue(c_intakeCommand.setIntakeState(IntakeStates.UP));
        // driver1.x().onFalse(c_intakeCommand.setIntakeState(IntakeStates.STOP));

        // driver1.b().onTrue(c_intakeCommand.setIntakeState(IntakeStates.DOWN));
        // driver1.b().onFalse(c_intakeCommand.setIntakeState(IntakeStates.STOP));
        
        // Bind Intake Commands
        // driver1.x().onTrue(new InstantCommand(() -> s_intake.m_pivot.set(0.4)));
        // driver1.x().onFalse(new InstantCommand(() -> s_intake.m_pivot.set(0)));

        // driver1.b().onTrue(new InstantCommand(() -> s_intake.m_pivot.set(-0.6)));
        // driver1.b().onFalse(
        //     new InstantCommand(() -> s_intake.m_pivot.set(0)));        

        // driver1.pov(270).onTrue(c_intakeCommand.setIntakeState(IntakeStates.DOWN));
        
        // driver1.pov(270).onTrue(new InstantCommand(()-> s_intake.setSetpoint(-150)));
        // driver1.pov(270).onFalse(new InstantCommand(() -> s_intake.setSetpoint(-200)));


        // driver1.pov(0).onTrue(c_intakeCommand.setIntakeState(IntakeStates.UP)); 
        // driver1.pov(270).onTrue(c_intakeCommand.setIntakeState(IntakeStates.DOWN));

        // Bind Intake Controls
        driver1.leftTrigger().onTrue(c_intakeCommand.setIntakeState(IntakeStates.INTAKE));
        driver1.leftTrigger().onFalse(c_intakeCommand.setIntakeState(IntakeStates.STOP));

        driver1.leftBumper().toggleOnTrue(c_intakeCommand.setIntakeState(IntakeStates.REVERSE));
        driver1.leftBumper().toggleOnFalse(c_intakeCommand.setIntakeState(IntakeStates.STOP));

        // // Bind Indexor Commands
        driver1.rightBumper().onTrue(c_indexCommand.setIndexState(IndexStates.INDEX));
        driver1.rightBumper().onFalse(c_indexCommand.setIndexState(IndexStates.STOP));

        driver1.leftBumper().toggleOnTrue(c_indexCommand.setIndexState(IndexStates.REVERSE));
        driver1.leftBumper().toggleOnFalse(c_indexCommand.setIndexState(IndexStates.STOP));
    }

    public void setDriverBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-driver0.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
            //         .withVelocityY(-driver0.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
            //         .withRotationalRate(-driver0.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
            // )
            c_teleop
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver0.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver0.back().and(driver0.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver0.back().and(driver0.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver0.start().and(driver0.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver0.start().and(driver0.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureAuto() {
        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(m_chooser);

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}