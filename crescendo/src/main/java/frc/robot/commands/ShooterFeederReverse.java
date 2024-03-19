// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterFeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.subsystems.LineBreak;

/** An example command that uses an example subsystem. */
public class ShooterFeederReverse extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterFeederSubsystem m_sf_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterFeederReverse(ShooterFeederSubsystem sf_subsystem) {
    m_sf_subsystem = sf_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_sf_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_sf_subsystem.reverse();
  }
  // Called once the command ends or is interrupted.
  @Override
  
  public void end(boolean interrupted) {        // public void end(resting_bottom_bitch_state = false) {
    m_sf_subsystem.stop();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
