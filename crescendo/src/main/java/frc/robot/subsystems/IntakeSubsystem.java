// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Intake Subsystem - Feeds Notes */

  private CANSparkMax floor_feeder_motor;
  private SparkPIDController floor_feeder_pid;
  private RelativeEncoder floor_feeder_encoder;


  public IntakeSubsystem() {

    floor_feeder_motor = new CANSparkMax(IntakeConstants.MOTOR_INTAKE_FLOOR, MotorType.kBrushless);
    floor_feeder_encoder = floor_feeder_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    floor_feeder_pid = floor_feeder_motor.getPIDController();
    floor_feeder_pid.setFeedbackDevice(floor_feeder_encoder);

  }

  /**
   * @param speed
   * @param direction
   */
  public void feed(double speed, boolean direction) {
    if (direction) {
      floor_feeder_motor.set(speed);
    }
    else {
      floor_feeder_motor.set(-speed);
    }
  }

  public void stop() {
    floor_feeder_motor.set(0.0);
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
