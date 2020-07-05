/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LimelightData;

import org.usfirst.frc.team4999.utils.Utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDriveCommand extends CommandBase {
  private final DriveSubsystem drive_subsystem;
  private final Limelight limelight;

  private boolean met;

  private double maxTurnRequest = 0.25;

  public AutonDriveCommand(DriveSubsystem subsystem, Limelight llight) {
    drive_subsystem = subsystem;
    limelight = llight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightData data = limelight.getData();
    double turnRequest;
    double moveRequest;
    double distance;

    if (data.valid()) {
      turnRequest = Utils.map(data.xCoord(), -Limelight.RANGE_X, Limelight.RANGE_X, -maxTurnRequest, maxTurnRequest);
      moveRequest = Utils.map(data.dist(), -Limelight.RANGE_Y, Limelight.RANGE_Y, -1.0, 1.0);
      distance = data.dist();
      met = data.targetMet();

    } else {
      turnRequest = 0;
      moveRequest = 0;
      distance = 0;
      met = false;
    }
    System.out.format("Target Distance:%.02f\n", distance);
    // Don't move forward, for safety reasons.
    drive_subsystem.drive(0, turnRequest);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
