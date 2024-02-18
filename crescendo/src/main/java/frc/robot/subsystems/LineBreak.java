// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterFeederPickUp;
import frc.robot.subsystems.ShooterFeederSubsystem;


public class LineBreak extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

      public boolean resting_bottom_bitch_state = false;
      public boolean top_toe_hoe_state = false;

      DigitalInput resting_bottom_bitch = new DigitalInput(8);  
      DigitalInput top_toe_hoe = new DigitalInput(7);




public LineBreak() {





  }
 

// public void Sensor1 () {
//     if (LineBreakConstants.LineBreakState = false) {
       
//         System.out.println("Note In Shoote");
        
//     }
//     else {
     
//         System.out.println("Shooter Empty");
//         return;
//     }
//   }

//   public void stop() {
//     floor_feeder_motor.set(0.0);
//   }
  
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



      resting_bottom_bitch_state = resting_bottom_bitch.get();
      top_toe_hoe_state = top_toe_hoe.get();


      //System.out.println("Resting Bottom Bitch State:" + resting_bottom_bitch_state);
      //System.out.println("Top Toe Hoe State:" + top_toe_hoe_state);


      if (resting_bottom_bitch_state == false) {
        
       // System.out.println("Note In Shooter");

       // ShooterFeederPickUp.end();

        return;

      }
     
     else { 
     
        // System.out.println("Shooter Empty");
         return;
     }      

     }

    
  
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
