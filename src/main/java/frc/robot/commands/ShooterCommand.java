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
    m_subsystem.runShooter();
    m_subsystem.shoot();
    m_subsystem.stopGate();
    m_hood_subsystem.stowHood();
  }

  @Override
  public void execute() {
    if (m_controller.getShootPowerCells() > 0) {
      m_hood_subsystem.deployHood();
      while (!m_hood_subsystem.getFullyDeployed()) {
      }
      m_subsystem.runGate();
    } else if (m_controller.getShootPowerCells() == 0) {
      m_hood_subsystem.stowHood();
      m_subsystem.stopGate();
    } else if (m_controller.getPurgePowerCells()) {
      m_hood_subsystem.stowHood();
      m_subsystem.reverseGate();
      m_subsystem.runGate();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_hood_subsystem.stowHood();
    m_subsystem.stopAll();
  }
}