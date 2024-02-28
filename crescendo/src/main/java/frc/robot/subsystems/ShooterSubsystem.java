// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new ArmBase. */

  private TalonFX shooter_motor_a;
  private TalonFX shooter_motor_b;
  private boolean is_firing;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shooter_motor_a = new TalonFX(ShooterConstants.MOTOR_SHOOTER_A);
    shooter_motor_b = new TalonFX(ShooterConstants.MOTOR_SHOOTER_B);
    shooter_motor_b.setInverted(true);
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

  public void shoot() {
    is_firing = true;
    shooter_motor_a.set(0.90);
    shooter_motor_b.set(0.90);
  }
  public void suck(double speed) { 
    shooter_motor_a.set(speed);//0.85
    shooter_motor_b.set(speed);
  }

  public void stop() {
    is_firing = false;
    shooter_motor_a.set(0.0);
    shooter_motor_b.set(0.0);
  }
   public void shootAMP() {
    is_firing = true;
    shooter_motor_a.set(0.079);
    shooter_motor_b.set(0.079);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public boolean getIsFiring() {
    return is_firing;
  }
}
