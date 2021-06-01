package frc.robot.choosers;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.utils.ShuffleboardTabRegister.Tab;

public class PathChooser extends SendableChooser<String> {
  private static final String NAME = "Path";

  public PathChooser(String def, String... options) {
    super();

    setDefaultOption(def, def);
    if (options != null) {
      for (String path : options) {
        addOption(path, path);
      }
    }

    Tab.getTab(Tab.PATHWEAVER).add(NAME, this).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(0, 1)
        .withSize(4, 1);
  }
}
