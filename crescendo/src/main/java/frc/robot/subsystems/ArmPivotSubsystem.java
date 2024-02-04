// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPivotConstants;

public class ArmPivotSubsystem extends SubsystemBase {
  /** Pivots the arm. */

  private CANSparkMax arm_pivot_motor_a;
  private CANSparkMax arm_pivot_motor_b;
  private DutyCycleEncoder arm_pivot_encoder;
  private PIDController arm_pid_controller;
  private double desired_location;

  private int active_mode;

  public ArmPivotSubsystem() {
    // Define the motors via spark max interface.
    arm_pivot_motor_a = new CANSparkMax(ArmPivotConstants.MOTOR_ARM_PIVOT_A, MotorType.kBrushless);
    arm_pivot_motor_a.setIdleMode(IdleMode.kBrake);
    arm_pivot_motor_b = new CANSparkMax(ArmPivotConstants.MOTOR_ARM_PIVOT_B, MotorType.kBrushless);
    arm_pivot_motor_b.setIdleMode(IdleMode.kBrake);

    // Configure PID Controller.
    arm_pid_controller = new PIDController(
      0.1, // Proportional gain
      0.0, // Integral gain
      0.0  // Derivative gain
    );
    //arm_pid_controller.enableContinuousInput(-0.1, 0.1);
    //arm_pid_controller.setOutputRange(-0.1, 0.1);

    desired_location = ArmPivotConstants.POSITION_PID_STARTING;

    // Define through-bore encoder attached to hex-shaft.
    // ENCA - DIO 8
    // ENCB - DIO 9
    arm_pivot_encoder = new DutyCycleEncoder(9);
    arm_pivot_encoder.setDistancePerRotation(1);

  }


  /**
   * Puts the arm into the desired mode.
   */
  public void go_to_mode(int MODE) {
      if (MODE == ArmPivotConstants.POSITION_STARTING) {
        desired_location = ArmPivotConstants.POSITION_PID_STARTING;
      }
      else if (MODE == ArmPivotConstants.POSITION_INTAKE_FLOOR) {
        desired_location = ArmPivotConstants.POSITION_PID_INTAKE_FLOOR;
      }
      else if (MODE == ArmPivotConstants.POSITION_INTAKE_FEEDER) {
        desired_location = ArmPivotConstants.POSITION_PID_INTAKE_FEEDER;
      }
      else if (MODE == ArmPivotConstants.POSITION_TRAVEL) {
        desired_location = ArmPivotConstants.POSITION_PID_TRAVEL;
      }
      else {
          System.out.println("Unknown position defined.");
          return;
      }
  }

public void go_to_mode(int MODE, double POS) {
      if (MODE == ArmPivotConstants.POSITION_STARTING) {
        desired_location = ArmPivotConstants.POSITION_PID_STARTING;
      }
      else if (MODE == ArmPivotConstants.POSITION_INTAKE_FLOOR) {
        desired_location = ArmPivotConstants.POSITION_PID_INTAKE_FLOOR;
      }
      else if (MODE == ArmPivotConstants.POSITION_INTAKE_FEEDER) {
        desired_location = ArmPivotConstants.POSITION_PID_INTAKE_FEEDER;
      }
      else if (MODE == ArmPivotConstants.POSITION_TRAVEL) {
        desired_location = ArmPivotConstants.POSITION_PID_TRAVEL;
      }
      else {
          System.out.println("Unknown position defined.");
          return;
      }

  }

  public void stop() {
    arm_pivot_motor_a.set(0.0);
    arm_pivot_motor_b.set(0.0);
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
    System.out.println(arm_pivot_encoder.isConnected() + " get" + arm_pivot_encoder.get() + " getDist " + arm_pivot_encoder.getDistance() + " getAbsPos" + arm_pivot_encoder.getAbsolutePosition() + " getPosOffset " + arm_pivot_encoder.getPositionOffset());

    // Get the current position from the encoder
    double currentPosition = arm_pivot_encoder.get();

    // Calculate the error (the difference between setpoint and current position)
    double error = desired_location - currentPosition;

    // Calculate the PID output
    double unclampedOutput = arm_pid_controller.calculate(desired_location);

    // Set motor output.
    if (error < -0.01 || error > 0.01) {
      double flip = 1;
      if (error <= 0) {
        flip = -1;
      }

      double output = 0.0;
      if (unclampedOutput >= 0) {
          output = 0.05 + (unclampedOutput / 2.0);
          // Clamp the output to the specified range
          output = Math.max(0.05, Math.min(0.5, output));
      }
      else {
          output = -0.05 - (unclampedOutput / 2.0);
          // Clamp the output to the specified range
          output = Math.min(-0.05, Math.max(-0.5, output));
      }

      arm_pivot_motor_a.set(output * flip);
      arm_pivot_motor_b.set(output * flip);

      //System.out.println("\n\n\nDesired: " + desired_location + "\nRaw Output: " + unclampedOutput + "\nOutput: " + output + "\nError: " + error + "\n\n");
    }
    else {
      arm_pivot_motor_a.set(0.0);
      arm_pivot_motor_b.set(0.0);
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
