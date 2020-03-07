/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.usfirst.frc.team4999.controllers.LogitechF310;
import org.usfirst.frc.team4999.utils.MoPDP;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoStowClimberCommand;
import frc.robot.commands.AutonDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.conditioners.*;
import frc.robot.subsystems.FalconDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.conditioners.CurvesConditioner;
import frc.robot.subsystems.conditioners.DeadzoneConditioner;
import frc.robot.subsystems.conditioners.SpeedLimitConditioner;
import frc.robot.utils.JoystickAnalogButton;
import frc.robot.utils.MoPrefs;
import frc.robot.controllers.ControllerBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // ---------------------------------------Components----------------------------------------
  private final MoPDP powerDistributionPanel = new MoPDP();
  private final Limelight limelight = new Limelight();

  // ----------------------------------------Joysticks-----------------------------------------
  private final XboxController xbox = new XboxController(0);
  private final LogitechF310 f310 = new LogitechF310(2);
  private final ControllerBase mainController = new ControllerBase(xbox, f310);

  // ----------------------------------------Buttons------------------------------------------------
  private final JoystickButton intakeRollerFwdButton = new JoystickButton(f310, LogitechF310.Button.kBumperLeft.value);
  private final JoystickButton intakePistonToggle = new JoystickButton(f310, LogitechF310.Button.kB.value);

  private final JoystickButton climberStow = new JoystickButton(f310, 7); // TODO Pick a button and update number
  private final JoystickButton climberClimb = new JoystickButton(f310, 8); // TODO Pick a button and update number

  private final JoystickAnalogButton shooterShoot = new JoystickAnalogButton(xbox,
      XboxController.Axis.kRightTrigger.value); // Right trigger
  private final JoystickButton purge = new JoystickButton(xbox, XboxController.Button.kBumperLeft.value); // Left bumper

  private final JoystickButton spdLimitInc = new JoystickButton(xbox, XboxController.Button.kY.value); // Y
  private final JoystickButton spdLimitDec = new JoystickButton(xbox, XboxController.Button.kA.value); // A

  private final JoystickButton reverseRobot = new JoystickButton(xbox, XboxController.Button.kB.value);

  private final JoystickButton storageButton = new JoystickButton(f310, LogitechF310.Button.kX.value);

  // ----------------------------------------Conditioners--------------------------------------
  private final SpeedLimitConditioner speedLimitConditioner = new SpeedLimitConditioner();
  private final ReverseConditioner reverseConditioner = new ReverseConditioner();
  private final DriveConditioner driveConditioner = new ComposedConditioner(new DeadzoneConditioner(),
      new CurvesConditioner(), reverseConditioner, speedLimitConditioner);

  // ---------------------------------------Subsystems----------------------------------------
  private final FalconDriveSubsystem falconDriveSubsystem = new FalconDriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(powerDistributionPanel);
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ShooterHoodSubsystem shooterHoodSubsystem = new ShooterHoodSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(shooterHoodSubsystem);
  private final StorageSubsystem storageSubsystem = new StorageSubsystem(powerDistributionPanel);
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  // ---------------------------------------Commands--------------------------------------------
  private final AutonDriveCommand autonDriveCommand = new AutonDriveCommand(falconDriveSubsystem, limelight);
  private final Command autonomousCommand = new ParallelCommandGroup(autonDriveCommand,
      new AutoStowClimberCommand(climberSubsystem));

  private final DriveCommand driveCommand = new DriveCommand(falconDriveSubsystem, mainController, driveConditioner);
  private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, shooterHoodSubsystem,
      mainController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    MoPrefs.safeForPrefs();

    // Configure the button bindings
    configureButtonBindings();

    // Set default commands as needed
    intakeSubsystem.setDefaultCommand(new RunCommand(intakeSubsystem::idle, intakeSubsystem));
    climberSubsystem.setDefaultCommand(new RunCommand(climberSubsystem::stop, climberSubsystem));
    // shooterSubsystem.setDefaultCommand(shooterCommand);
    storageSubsystem.setDefaultCommand(new RunCommand(storageSubsystem::stop, storageSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // -------------------------------------Intake-------------------------------------------------
    intakeRollerFwdButton.whileHeld(new InstantCommand(intakeSubsystem::runIntake, intakeSubsystem))
        .whenReleased(intakeSubsystem::idle, intakeSubsystem);
    intakePistonToggle.whenPressed(new InstantCommand(intakeSubsystem::toggleIntakeDeploy, intakeSubsystem));

    // -------------------------------------Climber----------------------------------------------
    climberStow.whileHeld(new InstantCommand(climberSubsystem::stow, climberSubsystem));
    climberClimb.whileHeld(new InstantCommand(climberSubsystem::climb, climberSubsystem));

    // --------------------------------------Drive--------------------------------------------------
    spdLimitInc.whenPressed(speedLimitConditioner::incSpeedLimit);
    spdLimitDec.whenPressed(speedLimitConditioner::decSpeedLimit);
    reverseRobot.whenPressed(reverseConditioner::toggleReversed);

    // --------------------------------------Storage------------------------------------------------

    // shooterShoot.whenPressed(new InstantCommand(shooterSubsystem::shoot,
    // shooterSubsystem, shooterHoodSubsystem));
    // Purge should also reverse storage and intake. Need to work out best way to do
    // that.
    purge.whenPressed(new InstantCommand(shooterSubsystem::purge, shooterSubsystem, shooterHoodSubsystem));

    storageButton.whileHeld(storageSubsystem::run, storageSubsystem);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousCommand;
  }

  public Command getTeleopCommand() {
    return driveCommand;
  }

  public Command getShooterCommand() {
    return shooterCommand;
  }
}
