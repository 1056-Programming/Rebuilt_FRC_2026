package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Utilities;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class OdometryAlign extends Command {
  private final PIDController xController, yController, rotController;
  private final SwerveRequest.RobotCentric robotCentricDrive = 
     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final CommandSwerveDrivetrain drivetrain;

  public OdometryAlign(CommandSwerveDrivetrain drivetrain, double x, double y, double r) {
    xController = new PIDController(2.1, 0, 0);  // Vertical movement
    yController = new PIDController(2.1, 0, 0);  // Horitontal movement
    rotController = new PIDController(0.7, 0, 0);  // Rotation
    this.drivetrain = drivetrain;

    xController.setSetpoint(x);
    yController.setSetpoint(y);
    rotController.setSetpoint(r);

    rotController.enableContinuousInput(0, 360);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
      Pose2d positions = drivetrain.getState().Pose;

      double xSpeed = Utilities.clampTalonVoltage(-xController.calculate(positions.getX()));
      double ySpeed = Utilities.clampTalonVoltage(-yController.calculate(positions.getY()));
      double rotValue = rotController.calculate(Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));

      SmartDashboard.putNumber("X Target Offset", positions.getX());
      SmartDashboard.putNumber("Xspeed", xSpeed);
      SmartDashboard.putNumber("Calculated yaw", Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
      SmartDashboard.putNumber("Rspeed", rotValue);

      drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
        .withVelocityY(ySpeed).withRotationalRate(rotValue)).execute();

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
      .withVelocityY(0).withRotationalRate(0)).execute();
  }

  @Override
  public boolean isFinished() { 
    if(xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint()) {
        return true; 
    }
    return false;
  }
}