/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.controllers.ControllerBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax mortorSparkMAX = new CANSparkMax(SPARKMAX_SHOOTER_CAN_ADDR);

  public ShooterSubsystem() {
    if (ControllerBase.getShootPowerCellsOuter()) {
      shoot();
    }
  }

  public void shoot() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
