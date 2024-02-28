package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDBlueNo extends Command {
  /*
   * DIO LED Blue Disable Command
   * ----------------------------
   * 
   * This command hooks up to the LED subsystem and sets
   * the color to blue.
   */

  private final LEDSubsystem m_subsystem;

  public LEDBlueNo(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.set_blue(false);
  }
}
