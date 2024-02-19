//Copyed from team 4829
//
//


// package frc.robot.commands;


// import com.pathplanner.lib.commands.PathPlannerAuto;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.*;
// import frc.robot.commands.*;
// import frc.robot.subsystems.*;

// /**
//  * This class is for the 5 Ball Auto Command
//  */
// public class Auto_B_N2 extends SequentialCommandGroup {

//   public Auto_B_N2(ShooterSubsystem shooter, ArmPivotSubsystem armpivot,
//       IntakeSubsystem intake, ShooterFeederSubsystem shooterfeeder, ShooterSubsystem shoot,SwerveBase swerveBase) {

//     addCommands(
//         // 1. Backs up from the pad and intakes
//         new ParallelCommandGroup(
//             new FollowTrajectory(AutoBuilder.buildAuto("New Auto"), true),
//             new IntakeSubsystem(intake)
//         ).withTimeout(1.4),

//         // 2. Revs up the shooter while going in right in front of the third ball
//         new ParallelRaceGroup(
//             new FollowTrajectory(driveSubsystem, PathWeaverConstants.secondPath5Ball,
//                 false).withTimeout(2.1), // 2.36
//             new AutoRev(shooterSubsystem, LimelightSubsystem.getInstance(), ledsSubsystem),
//             new TowerIntake(towerSubsystem)
//         ),

//         // 3. Shoots the two balls it is currently holding then intakes the third ball
//         new AutoShootIntake(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
//             driveSubsystem, ledsSubsystem, intakeSubsystem).withTimeout(3.45),

//         // 4. Goes to the area where it can pick up cargo from human player while intaking
//         new ParallelCommandGroup(
//             new FollowTrajectory(driveSubsystem, PathWeaverConstants.thirdPath5Ball, false),
//             new IntakeWithTower(intakeSubsystem, towerSubsystem)
//         ).withTimeout(3.6),

//         // 5. Revs up shooter while going closer to the hoop
//         new ParallelCommandGroup(
//             new IntakeWithTower(intakeSubsystem, towerSubsystem),
//             new FollowTrajectory(driveSubsystem, PathWeaverConstants.fourthPath5Ball, false),
//             new RunCommand(() -> shooterSubsystem.setShooterRPM(
//                 ShooterConstants.bottomMotorValues[2][1], // Sets the RPMs for 8.5 feet away
//                 ShooterConstants.topMotorValues[2][1]
//             ))
//         ).withTimeout(2.5),

//         // 6. Sets the gyro offset
//         new InstantCommand(() -> driveSubsystem.setGyroOffset(AutoConstants.fiveBallAutoOffset)),

//         // 7. Shoots the two balls gotten from the human player and corner
//         new AutoShoot(shooterSubsystem, towerSubsystem, LimelightSubsystem.getInstance(),
//             driveSubsystem, ledsSubsystem)

//     );

//   }

// }