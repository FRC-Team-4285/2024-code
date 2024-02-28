package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDWinNo extends Command {
  /*
   * DIO LED Win Disable Command
   * ---------------------------
   * 
   * This command hooks up to the LED subsystem and sets
   * the color to win.
   */

  private final LEDSubsystem m_subsystem;

  public LEDWinNo(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.set_win(false);
  }
}
