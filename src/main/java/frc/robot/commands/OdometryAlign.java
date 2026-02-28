// package frc.robot.commands.autoCommands;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.util.Utilities;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class autoPhotonAlign extends Command {
//   private final PIDController xController, yController, rotController;
//   private final SwerveRequest.RobotCentric robotCentricDrive = 
//      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//   private final CommandSwerveDrivetrain drivetrain;

//   private double startTime; 

//   private Timer dontSeeTagTimer, stopTimer;


//   public autoPhotonAlign(CommandSwerveDrivetrain drivetrain) {
//     xController = new PIDController(2.1, 0, 0);  // Vertical movement
//     yController = new PIDController(2.1, 0, 0);  // Horitontal movement
//     rotController = new PIDController(0.7, 0, 0);  // Rotation
//     this.drivetrain = drivetrain;
    
//    // addRequirements(drivetrain, camera);
//     rotController.enableContinuousInput(0, 360);

//   }

//   @Override
//   public void initialize() {
//     this.stopTimer = new Timer();
//     this.stopTimer.start();

//     this.dontSeeTagTimer = new Timer();
//     this.dontSeeTagTimer.start();
//     this.startTime = Timer.getFPGATimestamp();

//     rotController.setSetpoint(this.desiredYaw);
//     SmartDashboard.putNumber("Desired Yaw", this.desiredYaw);

//     rotController.setTolerance(PhotonConsts);

//     xController.setSetpoint(PhotonConsts.CENTER_TO_TAG_DELTA_X_DEFAULT);

//     yController.setSetpoint(yOffset);

//   }

//   @Override
//   public void execute() {
//     if (camera) {
//       this.dontSeeTagTimer.reset();
//       Transform3d positions = camera.getBestTagToCamera();
//       double xSpeed = -xController.calculate(positions.getX());
//       double ySpeed = -yController.calculate(positions.getY());
//       double rotValue = rotController.calculate(Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));

//       SmartDashboard.putNumber("X Target Offset", positions.getX());
//       SmartDashboard.putNumber("Xspeed", xSpeed);
//       SmartDashboard.putNumber("Calculated yaw", Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
//       SmartDashboard.putNumber("Rspeed", rotValue);

//       drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
//         .withVelocityY(ySpeed).withRotationalRate(rotValue)).execute();

//       //reset to 0 if not at desired position
//       if (!rotController.atSetpoint() ||
//           !yController.atSetpoint() ||
//           !xController.atSetpoint()) {
//         stopTimer.reset();
//       }
//     } else {
//       drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
//         .withVelocityY(0).withRotationalRate(0)).execute(); 
//     }

//     SmartDashboard.putNumber("Pose Validation Timer", stopTimer.get());
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
//       .withVelocityY(0).withRotationalRate(0)).execute();
//   }

//   @Override
//   public boolean isFinished() { 
//     if(xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint()) {
//         return true; 
//     }
//   }
// }