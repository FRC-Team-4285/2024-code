package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveBase;

public class AlignPoseAmp extends Command {
  /*
   * Align to Pose Swerve Drive Command
   * ----------------------------------
   * 
   * This command hooks up to the Swerve Drive subsystem
   * and aligns our robot to the speaker.
   */

  private final SwerveBase m_drive;

  public AlignPoseAmp(SwerveBase subsystem) {
    m_drive = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {

  }
}
