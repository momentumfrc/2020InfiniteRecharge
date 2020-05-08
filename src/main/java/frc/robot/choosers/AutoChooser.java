package frc.robot.choosers;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AutoChooser extends SendableChooser<Command> {
  private static final String NAME = "AUTONOMOUS";

  public AutoChooser(ShuffleboardTab tab, Command visionDrive, Command driveToWall, Command shootFromLine,
      Command shootFromWall) {
    super();

    setDefaultOption("Drive To Wall", driveToWall);
    addOption("Vision Drive", visionDrive);
    addOption("Shoot From Line", shootFromLine);
    addOption("Shoot From Wall", shootFromWall);

    tab.add(NAME, this).withWidget(BuiltInWidgets.kSplitButtonChooser);
  }
}