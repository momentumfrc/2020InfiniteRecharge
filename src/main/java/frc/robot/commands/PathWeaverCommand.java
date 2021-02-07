package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathWeaverCommand extends CommandBase {
  private static final String TRAJECTORY_PATH = "paths/test.path";
  private Trajectory trajectory;
  private double startTime = 0;
  private final RamseteController controller = new RamseteController();

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
    if (startTime == 0) {
      startTime = Timer.getFPGATimestamp();
    }
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    Trajectory.State goal = trajectory.sample(elapsedTime);
    controller.calculate(new Pose2d() /* FIXME Replace this with the real robot pose */, goal);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
