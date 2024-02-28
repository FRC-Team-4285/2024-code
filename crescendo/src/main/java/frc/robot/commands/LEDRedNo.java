package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDRedNo extends Command {
  /*
   * DIO LED Red Disable Command
   * ---------------------------
   * 
   * This command hooks up to the LED subsystem and sets
   * the color to red.
   */

  private final LEDSubsystem m_subsystem;

  public LEDRedNo(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.set_red(false);
  }
}
