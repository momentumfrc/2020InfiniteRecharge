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
import com.ctre.phoenix.motorcontrol.SensorCollection;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;

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

  private final PIDController movePID;
  private final PIDController turnPID;

  public DriveSubsystem() {
    // Slaves the left rear motor to the left front motor
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Initializes the 2 PIDControllers that will generate appropriate motor power
    // values
    movePID = new PIDController(1, 1, 1);
    turnPID = new PIDController(1, 1, 1);

    // Links both master-slave groups
    drive = new DifferentialDrive(leftFront, rightFront);

    drive.setDeadband(0);
  }

  public void arcadeDrive(double moveRequest, double turnRequest, double speedLimiter) {
    /*
     * try(MoPerfMon.Period period =
     * Robot.perfMon.newPeriod("DriveSubsystem::arcadeDrive")) {
     */

    // Scales the move and turn targets so they don't exceed the speed limit.
    moveRequest *= speedLimiter;
    turnRequest *= speedLimiter;

    // Gets the actual forward/backward motion of the robot.
    double moveRate = getMoveRate();
    // Gets the acutal speed difference between left and right sides of the robot.
    double turnRate = getTurnRate();

    /**
     * Generates a motor power value that will move the robot where the driver wants
     * it to go, based on the difference between the current motor rate and the
     * target.
     */
    double move = pidEnabled ? movePID.calculate(moveRate, moveRequest) : moveRequest;
    double turn = pidEnabled ? turnPID.calculate(turnRate, turnRequest) : turnRequest;

    // Forces the speed limit on the output of the PID controllers.
    double m_r = Utils.clip(move, -speedLimiter, speedLimiter);
    double t_r = Utils.clip(turn, -speedLimiter, speedLimiter);

    // Sends the actual drive command that activates the motors.
    drive.arcadeDrive(m_r, t_r, false);
    // }
  }

  public void resetEncoders() {
    leftEnc.setQuadraturePosition(0, 0); // Resets both encoders so they count from zero.
    rightEnc.setQuadraturePosition(0, 0);
  }

  public double getMoveRate() {
    return (leftEnc.getQuadratureVelocity() + rightEnc.getQuadratureVelocity()) / 2; // Averages the speed of both drive
                                                                                     // sides to get mean forward
                                                                                     // velocity.
  }

  public double getTurnRate() {
    return (leftEnc.getQuadratureVelocity() - rightEnc.getQuadratureVelocity()); // Subtracts the speed of both drive
                                                                                 // sides to get net turn vector.
  }

  public void stop() {
    drive.arcadeDrive(0, 0, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void useOutput(double output, double setpoint) {

  };

  public double getMeasurement() {
    return 0;
  }
}
