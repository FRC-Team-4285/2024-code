// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPivotConstants;

public class ArmPivotSubsystem extends SubsystemBase {
  /** Pivots the arm. */

  private CANSparkMax arm_pivot_motor_a;
  private CANSparkMax arm_pivot_motor_b;
  private DutyCycleEncoder arm_pivot_encoder;

  private int active_mode;

  public ArmPivotSubsystem() {
    // Define the motors via spark max interface.
    arm_pivot_motor_a = new CANSparkMax(ArmPivotConstants.MOTOR_ARM_PIVOT_A, MotorType.kBrushless);
    arm_pivot_motor_a.setIdleMode(IdleMode.kBrake);
    arm_pivot_motor_b = new CANSparkMax(ArmPivotConstants.MOTOR_ARM_PIVOT_B, MotorType.kBrushless);
    arm_pivot_motor_b.setIdleMode(IdleMode.kBrake);

    // Define through-bore encoder attached to hex-shaft.
    // ENCA - DIO 8
    // ENCB - DIO 9

    arm_pivot_encoder = new DutyCycleEncoder(9);
    // arm_pivot_encoder.setPositionOffset(
    //   arm_pivot_encoder.getPositionOffset()
    // );
    arm_pivot_encoder.setDistancePerRotation(0.5);

    //arm_pivot_encoder.reset();

    // System.out.println(arm_pivot_encoder.isConnected() + " get" + arm_pivot_encoder.get() + " getDist " + arm_pivot_encoder.getDistance() + " getAbsPos" + arm_pivot_encoder.getAbsolutePosition());
    // armPivotMotorEncoder = armPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // armPivotMotorEncoder.setPositionConversionFactor(0.1);
    // armPivotMotorRelEncoder = armPivotMotorB.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
    // armPivotMotorPID = armPivotMotorB.getPIDController();
    // armPivotMotorPID.setFeedbackDevice(armPivotMotorRelEncoder);

  }


  /**
   * Puts the arm into the desired mode.
   */
  public void go_to_mode(int MODE) {
      // Are we already going to this position?
      // if (active_mode == MODE) {
      //     return;
      // }
      if (MODE == ArmPivotConstants.POSITION_STARTING) {
        arm_pivot_motor_a.set(0.1);
        arm_pivot_motor_b.set(0.1);
      }
      else if (MODE == ArmPivotConstants.POSITION_FEED) {
        arm_pivot_motor_a.set(-0.1);
        arm_pivot_motor_b.set(-0.1);
      }
      else if (MODE == ArmPivotConstants.POSITION_TRAVEL) {
      }
      else {
          System.out.println("Unknown position defined.");
          return;
      }

      // Must set this at the very end.
      active_mode = MODE;
  }

public void go_to_mode(int MODE, double POS) {
      // Are we already going to this position?
      // if (active_mode == MODE) {
      //     return;
      // }

      if (MODE == ArmPivotConstants.POSITION_STARTING) {
        arm_pivot_motor_a.set(0.1);
        arm_pivot_motor_b.set(0.1);
      }
      else if (MODE == ArmPivotConstants.POSITION_FEED) {
        arm_pivot_motor_a.set(-0.1);
        arm_pivot_motor_b.set(-0.1);
      }
      else if (MODE == ArmPivotConstants.POSITION_TRAVEL) {
      }
      else {
          System.out.println("Unknown position defined.");
          return;
      }

      // Must set this at the very end.
      active_mode = MODE;

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
    //System.out.println(arm_pivot_encoder.isConnected() + " get" + arm_pivot_encoder.get() + " getDist " + arm_pivot_encoder.getDistance() + " getAbsPos" + arm_pivot_encoder.getAbsolutePosition() + " getPosOffset " + arm_pivot_encoder.getPositionOffset());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
