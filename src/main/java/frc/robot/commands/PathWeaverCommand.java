package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconDriveSubsystem;

public class PathWeaverCommand extends CommandBase {
  private static final String TRAJECTORY_PATH = "paths/test.path";
  private Trajectory trajectory;
  private double startTime = 0;
  private final FalconDriveSubsystem subsystem;
  private final RamseteController controller = new RamseteController();

  public PathWeaverCommand(FalconDriveSubsystem subsystem) {
    // Loads the PathWeaver trajectory into memory.
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(TRAJECTORY_PATH));
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + TRAJECTORY_PATH, e.getStackTrace());
    }

    this.subsystem = subsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // On the first scheduler cycle, update the start time with the current time
    if (startTime == 0) {
      startTime = Timer.getFPGATimestamp();
    }
    // Calculate the time since the start of trajectory tracking
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    // Gets the desired forward and angular velocity from the trajectory at the
    // current time
    Trajectory.State goal = trajectory.sample(elapsedTime);
    // Calculates the desired robot forward and angular velocity,
    // and passes it to the drive subsystem's PID controllers
    subsystem.drive(controller.calculate(subsystem.getPose(), goal));
  }

  @Override
  public void end(boolean interrupted) {
    // Resets the start time so that the RIO doesn't have to be rebooted to re-run
    // the auto
    startTime = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
