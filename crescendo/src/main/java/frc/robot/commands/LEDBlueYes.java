package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDBlueYes extends Command {
  /*
   * DIO LED Blue Enable Command
   * ---------------------------
   * 
   * This command hooks up to the LED subsystem and sets
   * the color to blue.
   */

  private final LEDSubsystem m_subsystem;

  public LEDBlueYes(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.set_blue(true);
  }
}
