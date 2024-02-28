package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDRedYes extends Command {
  /*
   * DIO LED Red Enable Command
   * --------------------------
   * 
   * This command hooks up to the LED subsystem and sets
   * the color to red.
   */

  private final LEDSubsystem m_subsystem;

  public LEDRedYes(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.set_red(true);
  }
}
