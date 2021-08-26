package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveSubsystem extends SubsystemBase {
  abstract public void drive(double moveRequest, double turnRequest);

  abstract public void tankDrive(double left, double right);

  abstract public void stop();
}