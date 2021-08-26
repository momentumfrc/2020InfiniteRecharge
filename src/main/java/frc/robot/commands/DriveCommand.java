/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.conditioners.DriveConditioner;
import frc.robot.utils.ShuffleboardTabRegister.Tab;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.DriveController;

public class DriveCommand extends CommandBase {
  private final DriveController m_controller;
  private final DriveSubsystem m_subsystem;
  private final DriveConditioner m_conditioner;
  private boolean tankDrive;

  public DriveCommand(DriveSubsystem subsystem, DriveController controller, DriveConditioner conditioner) {
    m_controller = controller;
    m_subsystem = subsystem;
    m_conditioner = conditioner;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    ShuffleboardTab tab = Tab.getTab(Tab.MATCH);

    if (tab != null) {
      NetworkTableEntry tankDriveChooser = tab.add("Tank Drive", true).withWidget(BuiltInWidgets.kToggleSwitch)
          .withPosition(7, 1).getEntry();
      tankDriveChooser.addListener(notice -> tankDrive = notice.value.getBoolean(),
          EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.tankDrive) {
      m_subsystem.tankDrive(m_controller.getLeftStick(), m_controller.getRightStick());
    } else {
      m_subsystem.drive(m_conditioner.conditionMove(m_controller.getMoveRequest()),
                        m_conditioner.conditionTurn(m_controller.getTurnRequest()));
    }
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
    m_subsystem.drive(0, 0);
  }
}
