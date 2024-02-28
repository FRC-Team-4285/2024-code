// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LineBreakConstants;


public class LineBreak extends SubsystemBase {

      public boolean resting_bottom_bitch_state = false;
      public boolean top_toe_hoe_state = false;

      DigitalInput bottom_sensor = new DigitalInput(LineBreakConstants.DIO_BOTTOM_SENSOR);  
      DigitalInput top_sensor = new DigitalInput(7);


  public LineBreak() {

  }
 
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run   
  }

  public Boolean getBottomState() {
    return bottom_sensor.get();
  }
  
  public Boolean getTopState() {
    return top_sensor.get();
  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
