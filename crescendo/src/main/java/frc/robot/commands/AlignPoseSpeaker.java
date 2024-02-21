package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveBase;

public class AlignPoseSpeaker extends Command {
  /*
   * Align to Pose Swerve Drive Command
   * ----------------------------------
   * 
   * This command hooks up to the Swerve Drive subsystem
   * and aligns our robot to the speaker.
   */

  private final SwerveBase m_drive;

  public AlignPoseSpeaker(SwerveBase subsystem) {
    m_drive = subsystem;

    // Where are we currently at?
    // PhotonUtils.estimateFieldToRobotAprilTag(null, null, null);

    // Generate path to arrive at the goal.

    // Follow the path
    // m_drive.
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    

  }
}
