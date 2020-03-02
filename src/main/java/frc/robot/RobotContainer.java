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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  public final DriveCommand driveCommand = new DriveCommand(driveSubsystem, this);
  private final AutonDriveCommand autonDriveCommand = new AutonDriveCommand(driveSubsystem);

  private XboxController xbox = new XboxController(0);
  private LogitechF310 f310 = new LogitechF310(2);

  public final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final ControllerBase mainController = new ControllerBase(xbox, f310);

  private final JoystickButton intakeRollerFwdButton = new JoystickButton(f310, 4/* LeftBumper */);
  private final JoystickButton intakeRollerFwdRevToggle = new JoystickButton(f310, 0/* X */);
  private final JoystickButton intakePistonToggle = new JoystickButton(f310, 2/* B */);

  private final JoystickButton climberStow = new JoystickButton(f310, 7); // Pick a button and update number
  private final JoystickButton climberClimb = new JoystickButton(f310, 8); // Pick a button and update number

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    MoPrefs.safeForPrefs();

    // Configure the button bindings
    configureButtonBindings();

    // Set default commands as needed
    climberSubsystem.setDefaultCommand(new InstantCommand(climberSubsystem::stop, climberSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intakeRollerFwdButton.whenPressed(new InstantCommand(intakeSubsystem::runIntake, intakeSubsystem))
        .whenReleased(new InstantCommand(intakeSubsystem::stopIntake, intakeSubsystem));
    intakeRollerFwdRevToggle.whenPressed(new InstantCommand(intakeSubsystem::reverseIntake, intakeSubsystem));
    intakePistonToggle.whenPressed(new InstantCommand(intakeSubsystem::toggleIntakeDeploy, intakeSubsystem));

    climberStow.whileHeld(new InstantCommand(climberSubsystem::stow, climberSubsystem));
    climberClimb.whileHeld(new InstantCommand(climberSubsystem::climb, climberSubsystem));
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
}
