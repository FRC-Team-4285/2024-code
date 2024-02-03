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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Intake Subsystem - Feeds Notes */

  private CANSparkMax armMotor_A;
  private SparkPIDController armMotorPID_A;
  private RelativeEncoder armMotorRelEncoder_A;

  private CANSparkMax armMotor_B;
  private SparkPIDController armMotorPID_B;
  private RelativeEncoder armMotorRelEncoder_B;
  
  private CANSparkMax armMotor_C;
  private SparkPIDController armMotorPID_C;
  private RelativeEncoder armMotorRelEncoder_C;
  private double desiredPosition;
  private boolean inPosition;

  public IntakeSubsystem() {

    armMotor_A = new CANSparkMax(IntakeConstants.MOTOR_INTAKE_CENTER, MotorType.kBrushless);
    armMotorRelEncoder_A = armMotor_A.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    armMotorPID_A = armMotor_A.getPIDController();
    armMotorPID_A.setFeedbackDevice(armMotorRelEncoder_A);

    armMotor_B = new CANSparkMax(IntakeConstants.MOTOR_INTAKE_EDGE_A, MotorType.kBrushless);
    armMotorRelEncoder_B = armMotor_B.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    armMotorPID_B = armMotor_B.getPIDController();
    armMotorPID_B.setFeedbackDevice(armMotorRelEncoder_B);
    
    armMotor_C = new CANSparkMax(IntakeConstants.MOTOR_INTAKE_EDGE_B, MotorType.kBrushless);
    armMotorRelEncoder_C = armMotor_C.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    armMotorPID_C = armMotor_C.getPIDController();
    armMotorPID_C.setFeedbackDevice(armMotorRelEncoder_C);

    desiredPosition = 99999; // starting value.
    inPosition = false;

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
