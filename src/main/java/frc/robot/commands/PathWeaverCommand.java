package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.choosers.PathChooser;
import frc.robot.subsystems.FalconDriveSubsystem;

public class PathWeaverCommand extends CommandBase {
  private Trajectory trajectory;
  private final FalconDriveSubsystem subsystem;
  private final RamseteController controller = new RamseteController();
  private boolean safeToChangePath = true;
  private PathChooser pathChooser;
  private Timer timer = new Timer();

  public PathWeaverCommand(FalconDriveSubsystem subsystem, PathChooser pathChooser) {
    this.subsystem = subsystem;
    this.pathChooser = pathChooser;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    safeToChangePath = false;
    // On the first scheduler cycle, update the start time with the current time
    if (timer.get() == 0) {
      timer.start();
    }
    // Gets the desired forward and angular velocity from the trajectory at the
    // current time
    Trajectory.State goal = trajectory.sample(timer.get());
    // Calculates the desired robot forward and angular velocity,
    // and passes it to the drive subsystem's PID controllers
    ChassisSpeeds speeds = controller.calculate(subsystem.getPose(), goal);
    subsystem.drive(speeds);
    System.out.println("vX: " + speeds.vxMetersPerSecond + " vY: " + speeds.vyMetersPerSecond + " omega: "
        + speeds.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    // Resets the start time so that the RIO doesn't have to be rebooted to re-run
    // the auto
    timer.stop();
    timer.reset();
    safeToChangePath = true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void updateTrajectory() {
    if (safeToChangePath) {
      String path = pathChooser.getSelected();
      // Loads the PathWeaver trajectory into memory. If reading the file is
      // unsuccessful, print to DS.
      if (path != null) {
        try {
          trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(path));
        } catch (IOException e) {
          DriverStation.reportError("Unable to open trajectory: " + path, e.getStackTrace());
        }
      } else {
        System.out.println("Trajectory path was null");
      }

    } else {
      System.out.println("Path cannot be changed during autonomous run!");
    }
  }
}
