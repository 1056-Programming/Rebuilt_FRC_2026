// package frc.robot.subsystems;


// import static edu.wpi.first.units.Units.Second;

// import java.util.function.Supplier;

// import edu.wpi.first.units.measure.Time;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.LEDPattern;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class AddressableLEDSubsystem extends SubsystemBase {
//     private final AddressableLED led;
//     private final AddressableLEDBuffer buffer;
//     private Supplier<Boolean> isAtRps;
//     private int shooterSpeed = 0;
//     private Color teamColor;



//     public AddressableLEDSubsystem(int port, int length, Supplier<Boolean> isAtRps) {
//         led = new AddressableLED(0);
//         buffer = new AddressableLEDBuffer(60);
//         this.isAtRps = isAtRps;
//         led.setLength(buffer.getLength());
//         led.start();

//         var alliance = DriverStation.getAlliance(); //Alliance color
//         teamColor = Color.kWhite;
//         // Color teamColor = (alliance == DriverStation.Alliance.Red) ? Color.kRed : Color.kBlue;
//         if (alliance.equals(DriverStation.Alliance.Red)) {
//             teamColor = Color.kRed;
//         } else if (alliance.equals(DriverStation.Alliance.Blue)) {
//             teamColor = Color.kBlue;
//         } else {
//             teamColor = Color.kWhite; // Default color if alliance is unknown
//         }
//     }


//     // @Override
//     // public void Init() {                
//     //     var alliance = DriverStation.getAlliance(); //Alliance color
//     //     Color teamColor = Color.kWhite;
//     //     // Color teamColor = (alliance == DriverStation.Alliance.Red) ? Color.kRed : Color.kBlue;
//     //     if (alliance.equals(DriverStation.Alliance.Red)) {
//     //         teamColor = Color.kRed;
//     //     } else if (alliance.equals(DriverStation.Alliance.Blue)) {
//     //         teamColor = Color.kBlue;
//     //     } else {
//     //         teamColor = Color.kWhite; // Default color if alliance is unknown
//     //     }
//     // }


//     @Override
//     public void periodic() { //Make sure running this 20 ms
//         // double desiredShooterRPS = ShooterSubsystem.desiredShooterRPS; //Desired RPS value
//         // double desiredBackSpinRPS = ShooterSubsystem.desiredBackSpinRPS;
//         // double[] shooterSpeeds = s_shooter.getMotorSpeeds(); // Get current shooter motor speeds
//         // double backSpinSpeed = s_shooter.getBackSpinRPS(); // Get current backspin speed


//         // s_AddressableLEDSubsystem = new AddressableLEDSubsystem(0, 60); //Call LED subsystem
//         // s_AddressableLEDSubsystem.setDefaultCommand( //Create Command
//         //     s_AddressableLEDSubsystem.run(() -> {
//         //         var alliance = DriverStation.getAlliance(); //Alliance color
//         //         double desiredShooterRPS = ShooterSubsystem.desiredShooterRPS;
//         //         double desiredBackSpinRPS = ShooterSubsystem.desiredBackSpinRPS;
//         //         double[] shooterSpeeds = s_shooter.getMotorSpeeds(); // Get current shooter motor speeds
//         //         double backSpinSpeed = s_shooter.getBackSpinRPS(); // Get current backspin speed
               


//         //         Color teamColor = Color.kWhite
//         //         // Color teamColor = (alliance == DriverStation.Alliance.Red) ? Color.kRed : Color.kBlue;
//         //         if (alliance == DriverStation.Alliance.Red) {
//         //             teamColor = Color.kRed;
//         //         } else if (alliance == DriverStation.Alliance.Blue) {
//         //             teamColor = Color.kBlue;
//         //         } else {
//         //             teamColor = Color.kWhite; // Default color if alliance is unknown
//         //         }


//         if (isAtRps.get()) { // Apply blinking pattern only when the RPS is max
//             LEDPattern.solid(teamColor).blink(Time.ofBaseUnits(0.2, Second)).Pattern.applyTo(());
//         } else {
//             // Idle state: set a solid color or turn off (Most likely just stay the team color unless match end)
//             LEDPattern.solid(teamColor);
//         }
//         led.setData(buffer); //Update hardware with changes
//     }
// }



