package frc.robot.choosers;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class PathChooser extends SendableChooser<String> {
  private static final String NAME = "Path";

  public PathChooser(ShuffleboardTab tab, String def, String... options) {
    super();

    setDefaultOption(def, def);
    if (options != null) {
      for (String path : options) {
        addOption(path, path);
      }
    }

    tab.add(NAME, this).withWidget(BuiltInWidgets.kSplitButtonChooser);
  }
}
