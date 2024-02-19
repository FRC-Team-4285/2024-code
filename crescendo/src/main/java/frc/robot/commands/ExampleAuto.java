// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.*;
// import frc.robot.subsystems.*;
// import frc.robot.commands.*;


// public class ExampleAuto extends CommandBase {
//   /*
//    * Autonomous Command
//    * ------------------
//    * 
//    * This command is a stub that runs during autonomous.
//    * Currently unused, will be filled soon.
//    */

//    private double startTime = 0.0;
//    private final SwerveBase drive;
//    private final AprilTagSubsystem aprilTags;
//    private final ArmPivotSubsystem pivot;
//    private final IntakeSubsystem feed;
//    private final LineBreak notedetection;
//    private final ShooterFeederSubsystem shooterfeeder;
//    private final ShooterSubsystem shooter;

//     public ExampleAuto(SwerveBase swerveBase, AprilTagSubsystem apriltags,
//      ArmPivotSubsystem armpivot,  IntakeSubsystem intake, LineBreak linebreak, ShooterFeederSubsystem  ShooterSubsystem ) {
//         drive = swerveBase;
//         apriltags = aprilTags;
//         pivot = armPivot;
//         feed = intake;

//         addRequirements(swerveBase, armBase, armBaseCone);
//     }

//     // Called just before this Command runs the first time
//     @Override
//     public void initialize() {
//         startTime = getCurrentTime();
//         drive.zeroPigeon();
//     }

//     private double getCurrentTime() {
//         /*
//         * Returns current time in milliseconds.
//         */

//         return System.currentTimeMillis();
//     }

//     private double getTimeSinceInitialized() {
//         return getCurrentTime() - startTime;
//     }

//     // Called repeatedly when this Command is scheduled to run
//    // @Override
//     // public void execute() {
//     //     double timeSinceInitialized = getTimeSinceInitialized();
//     //     //System.out.println(getTimeSinceInitialized());
//     //     if (timeSinceInitialized < 100) {
//     //         drive.drive(0.05, 0.0, 0, true);
//     //         armBaseCone.unlock_arm();
//     //     }
//     //     else if (timeSinceInitialized < 2000) {
//     //         armBaseCone.unlock_arm();
//     //         armBase.go_to_position(ArmConstants.DROP_POS);
//     //         drive.drive(0.0, 0.0, 0.0, true);
//     //     }
//     //     else if (timeSinceInitialized < 2500) {
//     //         armBase.go_to_position(ArmConstants.DROP_POS);
//     //         armBaseCone.release_cone();
//     //     }
//     //     else if (timeSinceInitialized < 5200) {
//     //         drive.drive(1.0, 0.2, 0.0, true);
//     //         armBase.go_to_position(ArmConstants.START_POS);
//     //     }
//     //     else if (timeSinceInitialized < 6500) { //was 6200
//     //         drive.drive(1.0, 0.0, 0.0, true);
//     //         armBase.go_to_position(ArmConstants.START_POS);
//     //     }
//     //     else { // stop
//     //         drive.drive(0.0, 0, 0, true);
//     //         armBase.go_to_position(ArmConstants.START_POS);
//     //     }
//     // }

//     // Make this return true when this Command no longer needs to run execute()
//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     // Called once after isFinished returns true
//     @Override
//     public void end(boolean isInterrupted) {
//     }
// }
