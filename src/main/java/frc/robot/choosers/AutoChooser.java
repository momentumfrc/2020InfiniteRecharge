package frc.robot.choosers;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ShuffleboardTabRegister.Tab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class AutoChooser extends SendableChooser<Command> {
  private static final String NAME = "AUTONOMOUS";

  public AutoChooser(Command defaultCommand, Command... commands) {
    super();

    setDefaultOption(defaultCommand.getName(), defaultCommand);
    for (Command command : commands) {
      addOption(command.getName(), command);
    }

    Tab.getTab(Tab.MATCH).add(NAME, this).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(0, 1).withSize(4,
        1);
  }
}