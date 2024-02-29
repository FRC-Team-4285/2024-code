package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class LEDSubsystem extends SubsystemBase {

  /** Creates a new LEDSubsystem. */

  private DigitalOutput led_is_blue;
  private DigitalOutput led_is_red;
  private DigitalOutput led_is_win;


  public LEDSubsystem() {
    led_is_blue = new DigitalOutput(LEDConstants.DIO_LED_IS_BLUE);
    led_is_red = new DigitalOutput(LEDConstants.DIO_LED_IS_RED);
    led_is_win = new DigitalOutput(LEDConstants.DIO_LED_WIN);

    led_is_blue.set(true);
    led_is_red.set(true);
    led_is_win.set(true);
  }

  @Override
  public void periodic() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            //System.out.println("Alliance Red");
            set_red(LEDConstants.DIO_ENABLE);
            set_blue(LEDConstants.DIO_DISABLE);
        }
        if (ally.get() == Alliance.Blue) {
            //System.out.println("Alliance Blue");
            set_red(LEDConstants.DIO_DISABLE);
            set_blue(LEDConstants.DIO_ENABLE);
        }
    }
    else {
        System.out.println("I'm unset!");
        set_red(LEDConstants.DIO_DISABLE);
        set_blue(LEDConstants.DIO_DISABLE);
        //set_win(LEDConstants.DIO_DISABLE);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void stop() {
    /*
     * Stubbed for now.
     */
  }

  public void set_blue(boolean blue_state) {
    led_is_blue.set(blue_state);
  }

  public void set_red(boolean red_state) {
    led_is_red.set(red_state);
  }

  public void set_win(boolean win_state) {
    led_is_win.set(win_state);
  }

}
