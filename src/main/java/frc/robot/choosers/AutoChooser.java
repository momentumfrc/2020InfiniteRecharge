package frc.robot.choosers;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.commands.AutonDriveCommand;
import frc.robot.commands.DriveToWall;
import frc.robot.commands.ShootFromLine;
import frc.robot.commands.ShootFromWall;

public class AutoChooser extends SendableChooser<Command> {
  private final String NAME = "AUTONOMOUS";

  public AutoChooser(ShuffleboardTab tab, AutonDriveCommand visionDrive, DriveToWall driveToWall,
      ShootFromLine shootFromLine, ShootFromWall shootFromWall) {
    super();

    setDefaultOption("Drive To Wall", driveToWall);
    addOption("Vision Drive", visionDrive);
    addOption("Shoot From Line", shootFromLine);
    addOption("Shoot From Wall", shootFromWall);

    tab.add(NAME, this).withWidget(BuiltInWidgets.kSplitButtonChooser);
  }
}