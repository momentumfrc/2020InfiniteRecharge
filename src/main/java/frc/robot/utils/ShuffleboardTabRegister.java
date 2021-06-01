package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class ShuffleboardTabRegister {
  public static enum Tab {
    MATCH("Match"), DEBUG("Debug"), LIMELIGHT("Limelight"), PATHWEAVER("PathWeaver");

    private String name;

    Tab(String tabName) {
      this.name = tabName;
    }

    public String getTabKey() {
      return this.name();
    }

    public static ShuffleboardTab getTab(Tab tab) {
      return Shuffleboard.getTab(tab.name);
    }
  }
}
