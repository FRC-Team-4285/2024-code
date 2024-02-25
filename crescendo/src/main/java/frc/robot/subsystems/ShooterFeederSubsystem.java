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
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterFeederConstants;

public class ShooterFeederSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax feeder_motor;
  private RelativeEncoder feeder_motor_encoder;
  private SparkPIDController feeder_motor_pid;
  private RobotContainer m_robot_container;
  private boolean reached_top_sensor;
  private boolean reached_bottom_sensor;

  public ShooterFeederSubsystem(RobotContainer robot_container) {
    m_robot_container = robot_container;
    reached_top_sensor = false;
    reached_bottom_sensor = false;

    feeder_motor = new CANSparkMax(
      ShooterFeederConstants.MOTOR_SHOOTER_FEEDER,
      MotorType.kBrushless
    );

    feeder_motor_encoder = feeder_motor.getAlternateEncoder(
      SparkMaxAlternateEncoder.Type.kQuadrature,
      4096
    );

    feeder_motor_pid = feeder_motor.getPIDController();

    feeder_motor_pid.setFeedbackDevice(
      feeder_motor_encoder
    );

  }

  /**
   * @param speed
   * @param direction
   */
  public void feed(int feed_type) {
    /*
     * Call this method periodically from your command.
     * 
     * This command will identify using the robot's sensor's how to
     * run the motors in the shooter to properly align a note in the unit.
     */

    boolean bottomState = m_robot_container.getLineBreakSubsystem().getBottomState();
    boolean topState = m_robot_container.getLineBreakSubsystem().getTopState();
    ShooterSubsystem shooter_subsystem = m_robot_container.getShooterSubsystem();
  //}

    if (shooter_subsystem.getIsFiring()) {
      return;
    }

    if (feed_type == ShooterFeederConstants.SHOOTER_FEEDER_STATE_FLOOR_FEEDING) {
      /* Check if bottom line sensor is obstructed */
      if (bottomState == ShooterFeederConstants.LINEBREAK_BLOCKED) {
        /* We are in this block if the line break sensor on the BOTTOM is blocked. */
        if (topState == ShooterFeederConstants.LINEBREAK_OPEN) {
          /*
            The bottom is blocked AND the top is NOT blocked.
            This is ideal from FLOOR FEEDING perspective, so
            stop motors and do nothing.
          */
          if (reached_top_sensor) {
            stop();
            return;
          }
          return;
        }
        else {
          /*
           * The bottom is blocked AND the top IS blocked. This is NOT ideal when
           * FLOOR FEEDING. So we must run the system slowly in reverse to get the
           * note into the optimal position.
           */
          reached_top_sensor = true;
          feeder_motor.set(-0.05);
          return;
        }
      }
      else {
        /*
         * We are in this block if the line break sensor on the BOTTOM is OPEN
         * and we are FLOOR FEEDING.
         */
        feeder_motor.set(0.4);
        reached_top_sensor = false;
      }
    }
    else {
      /*
       * We are in this block if we are HUMAN FEEDING.
       */

      if (topState == ShooterFeederConstants.LINEBREAK_OPEN && bottomState == ShooterFeederConstants.LINEBREAK_OPEN) {
        feeder_motor.set(-0.4);
        shooter_subsystem.suck(-0.15);
        reached_top_sensor = false;
      }
      else if (reached_top_sensor) {
        if (topState == ShooterFeederConstants.LINEBREAK_OPEN && bottomState == ShooterFeederConstants.LINEBREAK_BLOCKED) {
          stop();
          shooter_subsystem.stop();
        }
        else {
          feeder_motor.set(0.05);
          shooter_subsystem.stop();
        }
      }
      /* Check if bottom line sensor is obstructed */
      else if (bottomState == ShooterFeederConstants.LINEBREAK_BLOCKED && topState == ShooterFeederConstants.LINEBREAK_BLOCKED) {
        /* We are in this block if the line break sensor on the TOP is blocked. */
        feeder_motor.set(-0.15);
        shooter_subsystem.suck(-0.1);
      }
      else if (bottomState == ShooterFeederConstants.LINEBREAK_BLOCKED && topState == ShooterFeederConstants.LINEBREAK_OPEN) {
        /* We are in this block if the line break sensor on the bottom is blocked. */
        reached_top_sensor = true;
        shooter_subsystem.stop();
        feeder_motor.set(0.0);
      }
    }
  }

  /**
   * @param speed
   * @param direction
   */
  public void shoot(double speed, boolean direction) {
    /*
     * Call this method periodically from your command.
     */
      if (direction) {
      feeder_motor.set(speed);
    }
    else {
      feeder_motor.set(-speed);
    }
  }

  public void stop() {
    feeder_motor.set(0.0);
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
