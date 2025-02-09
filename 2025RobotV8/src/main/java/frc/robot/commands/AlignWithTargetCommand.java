// package frc.robot.commands;

// import frc.robot.subsystems.PhotonVisionSubsystem;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// import com.ctre.phoenix6.swerve.SwerveDrivetrain;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class AlignWithTargetCommand extends SubsystemBase {
//     private final PhotonVisionSubsystem photonVisionSubsystem;
//     private final CommandSwerveDrivetrain drivetrain;

//     public AlignWithTargetCommand(PhotonVisionSubsystem photonVisionSubsystem, SwerveDrivetrain drivetrain) {
//     this.photonVisionSubsystem = photonVisionSubsystem;
//     this.drivetrain = drivetrain;
//     addRequirements(photonVisionSubsystem, drivetrain); // Ensure both are subsystems
// }

//     }

//     @Override
//     public void execute() {
//         if (photonVisionSubsystem.hasTargets()) {
//             double yaw = photonVisionSubsystem.getAngleToBestTarget();
//             drivetrain.applyRequest(() -> 
//             drivetrain.drive(0, 0, -yaw * 0.1);// Example proportional control
//             );
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return Math.abs(photonVisionSubsystem.getAngleToBestTarget()) < 1.0; // Stop when aligned
//     }
// }