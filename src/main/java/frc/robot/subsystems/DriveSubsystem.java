/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import frc.robot.utils.pid;
import frc.robot.utils.Utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Need to use WPI_TalonFX so that DifferentialDrive will accept the motors.
  private final WPI_TalonFX leftFront = new WPI_TalonFX(FALCON_DRIVE_LEFT_FRONT_CAN_ADDR);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(FALCON_DRIVE_LEFT_REAR_CAN_ADDR);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(FALCON_DRIVE_RIGHT_FRONT_CAN_ADDR);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(FALCON_DRIVE_RIGHT_REAR_CAN_ADDR);
  private final DifferentialDrive drive;

  private final SensorCollection leftEnc = new SensorCollection(leftFront);
  private final SensorCollection rightEnc = new SensorCollection(rightFront);

  private final boolean pidEnabled = false;

  private final pid movePID = new pid();
  private final pid turnPID = new pid();

  public DriveSubsystem() {
    leftRear.follow(leftFront); // Slaves the left rear motor to the left front motor
    rightRear.follow(rightFront);

    drive = new DifferentialDrive(leftFront, rightFront); // Links both master-slave groups

    drive.setDeadband(0);
  }

  public void arcadeDrive(double moveRequest, double turnRequest, double speedLimiter) {
    /*
     * try(MoPerfMon.Period period =
     * Robot.perfMon.newPeriod("DriveSubsystem::arcadeDrive")) {
     */

    moveRequest *= speedLimiter;
    turnRequest *= speedLimiter;

    double moveRate = getMoveRate();
    double turnRate = getTurnRate();

    double move = pidEnabled ? movePID.calculate(moveRequest, moveRate) : moveRequest;
    double turn = pidEnabled ? turnPID.calculate(turnRequest, turnRate) : turnRequest;

    double m_r = Utils.clip(move, -speedLimiter, speedLimiter);
    double t_r = Utils.clip(turn, -speedLimiter, speedLimiter);
    drive.arcadeDrive(m_r, t_r, false);
    // }
  }

  public void resetEncoders() {
    leftEnc.setQuadraturePosition(0, 0);
    rightEnc.setQuadraturePosition(0, 0);
  }

  public double getMoveRate() {
    double moverate = (leftEnc.getQuadratureVelocity() + rightEnc.getQuadratureVelocity()) / 2;
    return moverate;
  }

  public double getTurnRate() {
    double turnrate = (leftEnc.getQuadratureVelocity() - rightEnc.getQuadratureVelocity());
    return turnrate;
  }

  public void stop() {
    arcadeDrive(0, 0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
