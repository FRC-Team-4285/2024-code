// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ArmPivotSubsystem m_ArmPivotSubsystem = Robot.m_armPivot;

  /* Human interfaces */
  private final Joystick driverJoystick;
  private final Joystick loupedeck;
  //private JoystickButton btn_arm_pivot_down;
  private JoystickButton btn_arm_pivot_up;
  private JoystickButton btn_shooter_feeder;
  private JoystickButton btn_floor_feeder;
  private JoystickButton btn_armP_pivot_stop;
 // private JoystickButton btn_amp_scoring_pos;
  private JoystickButton btn_shooting;
 private JoystickButton btn_human_feeder;
 private JoystickButton btn_store;
 private JoystickButton btn_shooting_without_cameras;
 private JoystickButton btn_shooting_without_cameras_stage_leg;
 private JoystickButton btn_shooting_without_cameras_2nd_stage_leg;
 private JoystickButton btn_amp_scoring_pos;
 
  /* Subsystems */
  public static SwerveBase m_swerveBase;

  /* Parent Class */
  private final Robot m_robot;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    driverJoystick = new Joystick(0);
    loupedeck =new Joystick(1);

    DoubleSupplier limit = () -> 0.55 - 0.45*driverJoystick.getRawAxis(SwerveConstants.sliderAxis);
    /*maps sliderAxis to be between 0.1 and 1.0*/
    DoubleSupplier stopRotation = () -> driverJoystick.getRawButton(12) ? 0.0 : 1.0;
    /* clamps rotation to zero if button 12 is pressed */
    BiFunction<Double, Double, Double> Clamp = (val,lim) -> (Math.abs(val) < lim) ? val : Math.copySign(lim,val);

    m_swerveBase = new SwerveBase();
    m_swerveBase.setDefaultCommand(
      new TeleopSwerve(
        m_swerveBase,
        () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
        () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
        () -> -Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.rotationAxis), limit.getAsDouble() * stopRotation.getAsDouble()),
        () -> !driverJoystick.getRawButton(1) //inverted=fieldCentric, non-inverted=RobotCentric
      )
    );

    // Configure the trigger bindings
    configureBindings();
  }

  public Joystick getJoystick() {
    return driverJoystick;
  }

  public Joystick getLoupedeck() {
    return loupedeck;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    // btn_arm_pivot_down = new JoystickButton(driverJoystick, 11);
    // btn_arm_pivot_down.whileTrue(new ArmPivotDown(m_robot.getArmPivotSubsystem()));

    // btn_arm_pivot_up = new JoystickButton(driverJoystick, 12);
    // btn_arm_pivot_up.whileTrue(new ArmPivotUp(m_robot.getArmPivotSubsystem()));

   
    //Floor Intake When Held
    btn_floor_feeder = new JoystickButton(driverJoystick, 6);
    btn_floor_feeder.whileTrue(new FloorFeederTest(m_robot.getIntakeSubsystem()));
    btn_floor_feeder.whileTrue(new ArmPivotErrected (m_robot.getArmPivotSubsystem()));
    btn_floor_feeder.whileTrue(new ShooterFeederPickUp(m_robot.getShooterFeederSubsystem()));
    //btn_floor_feeder.whileFalse(new ArmPivotStore (m_robot.getArmPivotSubsystem()));

    //Turns on Shooter Wheels when held
    btn_arm_pivot_up = new JoystickButton(driverJoystick, 9);
    btn_arm_pivot_up.whileTrue(new ShooterTest(m_robot.getShooterSubsystem()));

    //Will Push the Note at full speed into shooter
   // btn_shooter_feeder = new JoystickButton(driverJoystick, 10);
   // btn_shooter_feeder.whileTrue(new ShooterFeederFire(m_robot.getShooterFeederSubsystem()));

    btn_shooter_feeder = new JoystickButton(driverJoystick, 10);
    btn_shooter_feeder.whileTrue(new ShooterFeederAMP(m_robot.getShooterFeederSubsystem()));

    //Emergency Stop For Pivot
    btn_armP_pivot_stop = new JoystickButton(driverJoystick, 5);
    btn_armP_pivot_stop.toggleOnTrue(m_ArmPivotSubsystem.stopCommand());
    //Moves Pivit Based of of feild position
   // btn_shooting = new JoystickButton(driverJoystick, 1);
   // btn_shooting.whileTrue(new ArmPivotShooting (m_robot.getArmPivotSubsystem()));
    //Moves to get note from Feeder
    btn_human_feeder = new JoystickButton(driverJoystick, 7);
    btn_human_feeder.whileTrue(new ArmPivotHumanFeeder (m_robot.getArmPivotSubsystem()));
    btn_human_feeder.whileTrue(new ShooterRevers(m_robot.getShooterSubsystem()));
    //btn_human_feeder.whileFalse(new ArmPivotStore (m_robot.getArmPivotSubsystem()));
    btn_human_feeder.whileTrue(new ShooterFeederReverse(m_robot.getShooterFeederSubsystem()));
    //Moves Pivit to stored position
    btn_store = new JoystickButton(driverJoystick, 8);
    btn_store.whileTrue(new ArmPivotStore (m_robot.getArmPivotSubsystem()));

    btn_store = new JoystickButton(driverJoystick, 4);
    btn_store.whileTrue(new ArmPivotErrected (m_robot.getArmPivotSubsystem()));
    
    //btn_shooting_without_cameras = new JoystickButton(driverJoystick, 3);
    //btn_shooting_without_cameras.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    //btn_shooting_without_cameras.whileTrue(new ShootingWithoutCameras(m_robot.getArmPivotSubsystem()));
    //btn_shooting_without_cameras.whileTrue(new ShooterTest(m_robot.getShooterSubsystem()));
    
    //btn_shooting_without_cameras_stage_leg = new JoystickButton(driverJoystick, 11);
    //btn_shooting_without_cameras_stage_leg.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    //btn_shooting_without_cameras_stage_leg.whileTrue(new ShootingWithoutCamerasStageLeg(m_robot.getArmPivotSubsystem()));
    //btn_shooting_without_cameras_stage_leg.whileTrue(new ShooterTest(m_robot.getShooterSubsystem()));

    //btn_shooting_without_cameras_2nd_stage_leg = new JoystickButton(driverJoystick, 12);
    //btn_shooting_without_cameras_2nd_stage_leg.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    //btn_shooting_without_cameras_2nd_stage_leg.whileTrue(new ShootingWithoutCameras2ndStageLeg(m_robot.getArmPivotSubsystem()));
    //btn_shooting_without_cameras_2nd_stage_leg.whileTrue(new ShooterTest(m_robot.getShooterSubsystem()));

    btn_amp_scoring_pos = new JoystickButton(driverJoystick, 1);
    //btn_amp_scoring_pos.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    btn_amp_scoring_pos.whileTrue(new AMPScoringPos(m_robot.getArmPivotSubsystem()));
    btn_amp_scoring_pos.whileTrue(new ShooterAMP(m_robot.getShooterFeederSubsystem()));

    //Lets Pathplanner acsess commands
    NamedCommands.registerCommand("Shoot", (new ArmPivotShooting (m_robot.getArmPivotSubsystem())));
    NamedCommands.registerCommand("Store", (new ArmPivotStore (m_robot.getArmPivotSubsystem())));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return AutoBuilder.buildAuto("New Auto");
  }

  public SwerveBase getSwerveSubsytem() {
    return m_swerveBase;
  }
}
