/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import frc.robot.utils.Utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class DriveSubsystem extends SubsystemBase {
  // Need to use WPI_TalonFX so that DifferentialDrive will accept the motors.
  private final WPI_TalonFX leftFront = new WPI_TalonFX(FALCON_DRIVE_LEFT_FRONT_CAN_ADDR);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(FALCON_DRIVE_LEFT_REAR_CAN_ADDR);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(FALCON_DRIVE_RIGHT_FRONT_CAN_ADDR);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(FALCON_DRIVE_RIGHT_REAR_CAN_ADDR);

  private final SensorCollection leftEnc = new SensorCollection(leftFront);
  private final SensorCollection rightEnc = new SensorCollection(rightFront);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26));

  private final boolean pidEnabled = false;

  /**
   * @param ENC_TICKS_PER_FOOT The number of TalonFX encoder ticks per foot that
   *                           the robot drives on 6" wheels. Used to input ft/s
   *                           into WPI_TalonFX.set(VelocityControl, double
   *                           value), which takes its value in encoder ticks per
   *                           second.
   * 
   */
  private final double ENC_TICKS_PER_FOOT = 1304;

  public DriveSubsystem() {
    // Slaves the left rear motor to the left front motor
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);
  }

  public void drive(double moveRequest, double turnRequest) {
    var chassisSpeeds = new ChassisSpeeds(moveRequest, 0, turnRequest);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    double leftMPerS = wheelSpeeds.leftMetersPerSecond;
    double rightMPerS = wheelSpeeds.rightMetersPerSecond;
    double leftETPerS = FeetToEncTicks(
        Units.metersToFeet(leftMPerS) < 12 && Units.metersToFeet(leftMPerS) > -12 ? Units.metersToFeet(leftMPerS) : 12);
    double rightETPerS = FeetToEncTicks(
        Units.metersToFeet(rightMPerS) < 12 && Units.metersToFeet(rightMPerS) > -12 ? Units.metersToFeet(leftMPerS)
            : 12);
    leftFront.set(ControlMode.Velocity, leftETPerS);
    rightFront.set(ControlMode.Velocity, rightETPerS);
  }

  public void resetEncoders() {
    leftEnc.setQuadraturePosition(0, 0); // Resets both encoders so they count from zero.
    rightEnc.setQuadraturePosition(0, 0);
  }

  /**
   * Averages the speed of both drive sides to get mean forward velocity
   */

  public void stop() {
    leftFront.stopMotor();
    rightFront.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private double FeetToEncTicks(double ft) {
    return ft * ENC_TICKS_PER_FOOT;
  }

  public double getMeasurement() {
    return 0;
  }
}
