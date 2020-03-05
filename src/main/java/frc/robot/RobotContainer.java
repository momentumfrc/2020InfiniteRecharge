/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.usfirst.frc.team4999.controllers.LogitechF310;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoStowClimberCommand;
import frc.robot.commands.AutonDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveConditioner;
import frc.robot.subsystems.FalconDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.JoystickAnalogButton;
import frc.robot.utils.MoPrefs;
import frc.robot.controllers.ControllerBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final FalconDriveSubsystem falconDriveSubsystem = new FalconDriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ShooterHoodSubsystem shooterHoodSubsystem = new ShooterHoodSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(shooterHoodSubsystem);

  private final DriveConditioner driveConditioner = new DriveConditioner();

  private final XboxController xbox = new XboxController(0);
  private final LogitechF310 f310 = new LogitechF310(2);

  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final ControllerBase mainController = new ControllerBase(xbox, f310);

  private final DriveCommand driveCommand = new DriveCommand(falconDriveSubsystem, mainController, driveConditioner);
  private final AutonDriveCommand autonDriveCommand = new AutonDriveCommand(falconDriveSubsystem);

  private final JoystickButton intakeRollerFwdButton = new JoystickButton(f310, 4); // Left bumper
  private final JoystickButton intakePistonToggle = new JoystickButton(f310, 2); // B

  private final JoystickButton climberStow = new JoystickButton(f310, 7); // Pick a button and update number
  private final JoystickButton climberClimb = new JoystickButton(f310, 8); // Pick a button and update number

  private final JoystickAnalogButton shooterShoot = new JoystickAnalogButton(xbox,
      XboxController.Axis.kRightTrigger.value); // Right trigger
  private final JoystickButton purge = new JoystickButton(xbox, XboxController.Button.kBumperLeft.value); // Left bumper

  private final JoystickButton spdLimitInc = new JoystickButton(xbox, XboxController.Button.kY.value); // Y
  private final JoystickButton spdLimitDec = new JoystickButton(xbox, XboxController.Button.kA.value); // A

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    MoPrefs.safeForPrefs();

    // Configure the button bindings
    configureButtonBindings();

    // Set default commands as needed
    intakeSubsystem.setDefaultCommand(new InstantCommand(intakeSubsystem::idle, intakeSubsystem));
    climberSubsystem.setDefaultCommand(new InstantCommand(climberSubsystem::stop, climberSubsystem));
    shooterSubsystem
        .setDefaultCommand(new InstantCommand(shooterSubsystem::idle, shooterSubsystem, shooterHoodSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Intake
    intakeRollerFwdButton.whileHeld(new InstantCommand(intakeSubsystem::runIntake, intakeSubsystem));
    intakePistonToggle.whenPressed(new InstantCommand(intakeSubsystem::toggleIntakeDeploy, intakeSubsystem));

    climberStow.whileHeld(new InstantCommand(climberSubsystem::stow, climberSubsystem));
    climberClimb.whileHeld(new InstantCommand(climberSubsystem::climb, climberSubsystem));

    shooterShoot.whenPressed(new InstantCommand(shooterSubsystem::shoot, shooterSubsystem, shooterHoodSubsystem));
    // Purge should also reverse storage and intake. Need to work out best way to do
    // that.
    purge.whenPressed(new InstantCommand(shooterSubsystem::purge, shooterSubsystem, shooterHoodSubsystem));

    // Drive
    spdLimitInc.whenPressed(new InstantCommand(driveConditioner::incSpeedLimit));
    spdLimitDec.whenPressed(new InstantCommand(driveConditioner::decSpeedLimit));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new ParallelCommandGroup(autonDriveCommand, new AutoStowClimberCommand(climberSubsystem));
  }

  public Command getTeleopCommand() {
    return driveCommand;
  }
}
