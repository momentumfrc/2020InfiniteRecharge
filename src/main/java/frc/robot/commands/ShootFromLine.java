package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconDriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootFromLine extends CommandBase {
  private final FalconDriveSubsystem m_drive;
  private final ShooterSubsystem m_shooter;
  private final StorageSubsystem m_storage;
  private final IntakeSubsystem m_intake;

  public ShootFromLine(FalconDriveSubsystem subsystem, ShooterSubsystem shooter, StorageSubsystem storage,
      IntakeSubsystem intake) {
    m_drive = subsystem;
    m_shooter = shooter;
    m_storage = storage;
    m_intake = intake;

    addRequirements(subsystem, shooter, storage, intake);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double time = Timer.getMatchTime();
    m_intake.lowerIntake();
    if (time > 5) {
      m_storage.run();
      m_shooter.shoot();
    } else if (time < 5) {
      m_drive.drive(0.25, 0);
      m_storage.stop();
      m_shooter.idle();
    } else {
      m_drive.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {

  }
}