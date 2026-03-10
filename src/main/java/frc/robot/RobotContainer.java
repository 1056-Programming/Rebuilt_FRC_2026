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
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first
.wpilibj2.command.Command;
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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.YawTeleop;

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
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);

    //** Initialize Subsystems **//
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
   // private final ShooterSubsystem s_shooter = new ShooterSubsystem(() -> drivetrain.getState().Pose.getX(), () -> drivetrain.getState().Pose.getY());
    private final ShooterSubsystem s_shooter = new ShooterSubsystem(drivetrain);
    private final IndexSubsystem s_indexor = new IndexSubsystem();
    private final IntakeSubsystem s_intake = new IntakeSubsystem();
    private final VisionSubsystem s_VisionSubsystem = new VisionSubsystem(drivetrain,"limelight-hotrock");

    //** Initialize Commands and auto **//
    private final ShooterCommand c_shooterCommand = new ShooterCommand(s_shooter);
    private final IndexCommand c_indexCommand = new IndexCommand(s_indexor);
    private final IntakeCommand c_intakeCommand = new IntakeCommand(s_intake);
    private final SwerveTeleop c_teleop = new SwerveTeleop(drivetrain, driver0);   
    private final YawTeleop c_yawTeleop = new YawTeleop(drivetrain, driver0);
    private SendableChooser<Command> m_chooser;


    public RobotContainer() {
        // TODO change to specific auto positioning
        drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d()));
        drivetrain.getPigeon2().setYaw(0);

        configureBindings();
        setDriverBindings();
        configureAuto();
    }

    private void configureBindings() {     
        //setIntakeBindings();
        setIndexorBindings();
        //setShooterBindings();
        setTestBindings();

        
    }

    private void setIntakeBindings() {
        driver1.a().toggleOnTrue(c_intakeCommand.setIntakeState(IntakeStates.INTAKE));
        driver1.a().toggleOnFalse(c_intakeCommand.setIntakeState(IntakeStates.HOME));

        driver1.x().toggleOnTrue(c_intakeCommand.setIntakeState(IntakeStates.OUTAKE));
        driver1.x().toggleOnFalse(c_intakeCommand.setIntakeState(IntakeStates.HOME));

        driver1.y().toggleOnTrue(c_intakeCommand.setIntakeState(IntakeStates.GIGA_HOME)); 
    }

    private void setIndexorBindings() {
        driver1.leftBumper().onTrue(c_indexCommand.setIndexState(IndexStates.INDEX));
        driver1.leftBumper().onFalse(c_indexCommand.setIndexState(IndexStates.STOP));

        driver1.rightBumper().toggleOnTrue(c_indexCommand.setIndexState(IndexStates.REVERSE));
        driver1.rightBumper().toggleOnFalse(c_indexCommand.setIndexState(IndexStates.STOP));
    }

    private void setShooterBindings() {
        driver1.rightTrigger().onTrue(c_shooterCommand.setShooterState(States.ShooterStates.VARIABLE_SHOOT));
        driver1.rightTrigger().onFalse(c_shooterCommand.setShooterState(States.ShooterStates.STOP));

        driver1.leftTrigger().onTrue(c_shooterCommand.setShooterState(States.ShooterStates.IN_120));
        driver1.leftTrigger().onFalse(c_shooterCommand.setShooterState(ShooterStates.STOP));
    }

    private void setDriverBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(c_teleop);

        // Yaw align with tag when triggered
        // Return to normal after positioning
        driver0.rightTrigger().toggleOnTrue(c_yawTeleop);
        driver0.rightTrigger().toggleOnFalse(c_teleop);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver0.rightTrigger().whileTrue(drivetrain.applyRequest(() -> brake));
        driver0.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))
        ));

        // Run SysId routin zes when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver0.back().and(driver0.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver0.back().and(driver0.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver0.start().and(driver0.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver0.start().and(driver0.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void setTestBindings() {
        driver1.pov(90).onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS+=5, s_shooter.desiredBackSpinRPS)));
        driver1.pov(90).onFalse(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));

        driver1.pov(180).onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS+=5)));
        driver1.pov(180).onFalse(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));
        
        driver1.pov(270).onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS-=2.5, s_shooter.desiredBackSpinRPS)));
        driver1.pov(270).onFalse(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));

        driver1.b().onTrue(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS-=2.5)));
        driver1.b().onFalse(new InstantCommand(() -> s_shooter.setVelocitySetpoints(s_shooter.desiredShooterRPS, s_shooter.desiredBackSpinRPS)));
    }

    private void configureAuto() {
        NamedCommands.registerCommand("Intake Balls", c_intakeCommand.setIntakeState(IntakeStates.AUTO_INTAKE));
        NamedCommands.registerCommand("Intake Feed Out", c_intakeCommand.setIntakeState(IntakeStates.OUTAKE));
        NamedCommands.registerCommand("Intake Home", c_intakeCommand.setIntakeState(IntakeStates.HOME));
        NamedCommands.registerCommand("Intake Start Pos", c_intakeCommand.setIntakeState(IntakeStates.START));
        NamedCommands.registerCommand("Intake Stop", c_intakeCommand.setIntakeState(IntakeStates.STOP));
        NamedCommands.registerCommand("Intake Mega Home", c_intakeCommand.setIntakeState(IntakeStates.GIGA_HOME));

        NamedCommands.registerCommand("Shoot 120 IN", c_shooterCommand.setShooterState(ShooterStates.IN_120));
        NamedCommands.registerCommand("Shoot 100 IN", c_shooterCommand.setShooterState(ShooterStates.IN_100));

        NamedCommands.registerCommand("Index Balls", c_indexCommand.setIndexState(IndexStates.AUTO_INDEX));
        NamedCommands.registerCommand("Index Reverse", c_indexCommand.setIndexState(IndexStates.REVERSE));
        NamedCommands.registerCommand("Index Stop", c_indexCommand.setIndexState(IndexStates.STOP));
        NamedCommands.registerCommand("Index Auto Balls", c_indexCommand.setIndexState(IndexStates.AUTO_INDEX));

        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(m_chooser);

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}