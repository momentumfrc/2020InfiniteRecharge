/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final TalonFX leftFront = new TalonFX(FALCON_DRIVE_LEFT_FRONT_CAN_ADDR);
  private final TalonFX leftRear = new TalonFX(FALCON_DRIVE_LEFT_REAR_CAN_ADDR);
  private final TalonFX rightFront = new TalonFX(FALCON_DRIVE_RIGHT_FRONT_CAN_ADDR);
  private final TalonFX rightRear = new TalonFX(FALCON_DRIVE_RIGHT_REAR_CAN_ADDR);

  /**
   * Creates a new ExampleSubsystem.
   */
  public DriveSubsystem() {
    // TODO: combine Talons into master/slave groups.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
