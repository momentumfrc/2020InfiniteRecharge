/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.Limelight.LimelightData;

import org.usfirst.frc.team4999.utils.Utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDriveCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Limelight limelightSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterHoodSubsystem hoodSubsystem;
  private final StorageSubsystem storageSubsystem;

  private static final double[] trajectoryTable = {}; // TODO: Get empirical testing data for this table

  private boolean met;

  private double maxTurnRequest = 0.25;

  public AutonDriveCommand(DriveSubsystem subsystem, Limelight limelight, ShooterSubsystem shooter,
      ShooterHoodSubsystem hood, StorageSubsystem storage) {
    driveSubsystem = subsystem;
    limelightSubsystem = limelight;
    shooterSubsystem = shooter;
    hoodSubsystem = hood;
    storageSubsystem = storage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, limelightSubsystem, shooterSubsystem, hoodSubsystem, storageSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightData data = limelightSubsystem.getData();
    double turnRequest;
    double moveRequest;
    double distance;
    limelightSubsystem.lightsOn();

    if (data.valid()) {
      turnRequest = Utils.map(data.xCoord(), -Limelight.RANGE_X, Limelight.RANGE_X, -maxTurnRequest, maxTurnRequest);
      // Don't move forward, for safety reasons.
      moveRequest = 0 * Utils.map(data.dist(), -Limelight.RANGE_Y, Limelight.RANGE_Y, -1.0, 1.0);
      distance = data.dist();
      met = data.targetMet();
      driveSubsystem.drive(moveRequest, -turnRequest);
      if (met) {
        // Index of the array should be the measured range, returned value is the hood
        // setpoint.
        shooterSubsystem.shoot();
        storageSubsystem.run();
      } else {
        shooterSubsystem.idle();
        storageSubsystem.stop();
      }
    } else {
      turnRequest = 0;
      moveRequest = 0;
      distance = 0;
      met = false;
    }
    System.out.format("Target Distance:%.02f\n", distance);
  }

  public boolean isMet() {
    return met;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
