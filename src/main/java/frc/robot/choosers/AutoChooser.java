package frc.robot.choosers;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AutoChooser extends SendableChooser<Command> {
  private static final String NAME = "AUTONOMOUS";

  public AutoChooser(ShuffleboardTab tab, Command defaultCommand, Command... commands) {
    super();

    setDefaultOption(defaultCommand.getName(), defaultCommand);
    for (Command command : commands) {
      addOption(command.getName(), command);
    }

    tab.add(NAME, this).withWidget(BuiltInWidgets.kSplitButtonChooser);
  }
}