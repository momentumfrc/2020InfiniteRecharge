/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ConditionedDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.controllers.ControllerBase;

public class DriveCommand extends CommandBase {
  private final ControllerBase m_controller;
  private final ConditionedDriveSubsystem m_cdss;

  public DriveCommand(ConditionedDriveSubsystem cdss, ControllerBase controller) {
    m_controller = controller;
    m_cdss = cdss;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cdss);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cdss.mapToLimit(m_controller.getMoveRequest(), m_controller.getTurnRequest());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void stop() {
    m_cdss.mapToLimit(0, 0);
  }
}
