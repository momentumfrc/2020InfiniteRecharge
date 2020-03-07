package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.ControllerBase;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private final ShooterSubsystem m_subsystem;
  private final ControllerBase m_controller;
  private final ShooterHoodSubsystem m_hood_subsystem;

  public ShooterCommand(ShooterSubsystem subsystem, ShooterHoodSubsystem hoodSubsystem, ControllerBase controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    m_hood_subsystem = hoodSubsystem;
  }

  @Override
  public void initialize() {
    m_subsystem.idle();
  }

  @Override
  public void execute() {
    if (m_controller.getBoolShoot()) {
      m_subsystem.shoot();
    } // else if (m_controller.getShootPowerCells() == 0) {
      // m_subsystem.idle();
    // } // else if (m_controller.getPurgePowerCells()) {
    // m_subsystem.purge();
    // }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.idle();
  }
}