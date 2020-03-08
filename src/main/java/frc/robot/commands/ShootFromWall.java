package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconDriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class ShootFromWall extends CommandBase {
  private final FalconDriveSubsystem m_subsystem;
  private final ShooterSubsystem m_shooter;
  private final StorageSubsystem m_storage;

  public ShootFromWall(FalconDriveSubsystem subsystem, ShooterSubsystem shooter, StorageSubsystem storage) {
    m_subsystem = subsystem;
    m_shooter = shooter;
    m_storage = storage;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double time = Timer.getMatchTime();
    if (time > 10) {
      m_subsystem.drive(0.25, 0);
    } else {
      m_subsystem.stop();
    }
    if (time > 5) {
      m_storage.run();
    } else if (time > 1) {
      m_storage.stop();
    }
    if (time > 3) {
      m_shooter.shoot();
    } else if (time > 1) {
      m_shooter.idle();
    }
  }

  @Override
  public void end(boolean interrupted) {

  }
}