package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathWeaverCommand extends CommandBase {
  private static final String TRAJECTORY_PATH = "paths/test.path";
  private Trajectory trajectory;

  public PathWeaverCommand() {
    // Loads the PathWeaver trajectory into memory.
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(TRAJECTORY_PATH));
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + TRAJECTORY_PATH, e.getStackTrace());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO Get a Trajectory.State from the loaded trajectory, pass it to a
    // RamseteController, and pass the output to the drive subsystem.
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
