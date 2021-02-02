/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import org.usfirst.frc.team4999.utils.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.utils.MoPrefs;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class FalconDriveSubsystem extends DriveSubsystem {
  // Need to use WPI_TalonFX so that DifferentialDrive will accept the motors.
  private final WPI_TalonFX leftFront = new WPI_TalonFX(FALCON_DRIVE_LEFT_FRONT_CAN_ADDR);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(FALCON_DRIVE_LEFT_REAR_CAN_ADDR);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(FALCON_DRIVE_RIGHT_FRONT_CAN_ADDR);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(FALCON_DRIVE_RIGHT_REAR_CAN_ADDR);

  private boolean enablePID = false;

  /**
   * The number of TalonFX encoder ticks per meter that the robot drives on 6"
   * wheels. Used to input ft/s into WPI_TalonFX.set(VelocityControl, double
   * value), which takes its value in encoder ticks per second.
   */
  private static final double ENC_TICKS_PER_METER = 4278.215;
  /**
   * The maximum acceleration allowed by the Motion Magic control mode, in encoder
   * ticks per 100 ms per second. For example, a value of ~420 for this is roughly
   * equivalent to 1 m/s^2.
   */
  private static final int ACCELERATION_LIMIT = 854;
  /**
   * The number of inches between wheels on the drive base of the robot.
   */
  private static final double DRIVE_BASE_WIDTH_INCHES = 26;
  /**
   * The maximum individual wheel speed of the robot in m/s.
   */
  private static final double SPEED_LIMIT_METERS_PER_S = 4;
  /**
   * The maximum safe angular velocity of the robot, in radians per second.
   */
  private static final double TURN_LIMIT_RAD_PER_S = 4 * Math.PI;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(DRIVE_BASE_WIDTH_INCHES));

  private NetworkTableEntry leftDriveVelocity;
  private NetworkTableEntry rightDriveVelocity;

  private double moveReqScaled;
  private double turnReqScaled;
  private double leftETPerS;
  private double leftMPerS;

  public FalconDriveSubsystem(ShuffleboardTab tab) {
    // Invert one side of the robot
    // These should always be opposites
    // If the robot drives backwards, flip both
    leftFront.setInverted(false);
    leftRear.setInverted(false);
    rightFront.setInverted(true);
    rightRear.setInverted(true);
    // Set the braking mode
    leftFront.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    // Slaves the left rear motor to the left front motor
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);
    // Sets the PID configs for all motors.
    updatePIDConstants();
    // Sets the acceleration limit for all motors.
    leftFront.configMotionAcceleration(ACCELERATION_LIMIT);
    rightFront.configMotionAcceleration(ACCELERATION_LIMIT);

    NetworkTableEntry drivePIDchooser = tab.add("Drive PID", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    drivePIDchooser.addListener(notice -> enablePID = notice.value.getBoolean(),
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    leftDriveVelocity = tab.add("Drive Velocity L", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    rightDriveVelocity = tab.add("Drive Velocity R", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  }

  /**
   * @param moveRequest The "requested" forward motion as given by the controller
   *                    joystick, from -1 to 1.
   * @param turnRequest The "requested" angular motion as given by the controller
   *                    joystick, from -1 to 1.
   */
  public void drive(final double moveRequest, final double turnRequest) {
    if (enablePID) {
      /**
       * Since moveRequest is from -1 to 1 and we need a value in meters per second to
       * feed to ChassisSpeeds, we scale the moveRequest to the speed limit as
       * converted to m/s.
       */
      moveReqScaled = Utils.map(moveRequest, -1, 1, -SPEED_LIMIT_METERS_PER_S, SPEED_LIMIT_METERS_PER_S);
      /**
       * Since turnRequest is from -1 to 1 and we need a value in radians per second
       * to feed to ChassisSpeeds, we scale the turnRequest to the angular velocity
       * limit in radians/second.
       */
      turnReqScaled = Utils.map(turnRequest, -1, 1, -TURN_LIMIT_RAD_PER_S, TURN_LIMIT_RAD_PER_S);
      /**
       * The object that handles the calculations for how fast each side of the robot
       * should drive to accomplish the scaled move and turn requests.
       */
      final var chassisSpeeds = new ChassisSpeeds(moveReqScaled, 0, turnReqScaled);
      /**
       * The object that converts the overall chassis speed into individual wheel
       * speeds.
       */
      final DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
      /**
       * The variables that store the outputs of the wheel speeds, in meters per
       * second.
       */
      leftMPerS = wheelSpeeds.leftMetersPerSecond;
      final double rightMPerS = wheelSpeeds.rightMetersPerSecond;
      /**
       * The variables that store the wheel speeds in TalonFX encoder ticks per
       * second, as converted from m/s to encoder ticks/s.
       */
      leftETPerS = metersToEncTicks(leftMPerS);
      final double rightETPerS = metersToEncTicks(rightMPerS);
      leftFront.set(ControlMode.MotionMagic, leftETPerS);
      rightFront.set(ControlMode.MotionMagic, rightETPerS);
    } else {
      leftFront.set(ControlMode.PercentOutput, Utils.clip(moveRequest - turnRequest, -1, 1));
      rightFront.set(ControlMode.PercentOutput, Utils.clip(moveRequest + turnRequest, -1, 1));
    }
  }

  /**
   * Stops both motors.
   */
  public void stop() {
    leftFront.stopMotor();
    rightFront.stopMotor();
  }

  /**
   * 
   * @param m The number of meters to be converted to encoder ticks.
   * @return the number of encoder ticks that correspond to @param m , assuming
   *         6-inch wheels.
   */
  private double metersToEncTicks(final double m) {
    return m * ENC_TICKS_PER_METER;
  }

  /**
   * 
   * @param et The number of encoder ticks to be converted to meters.
   * @return The number of meters that correspond to @param m , assuming 6-inch
   *         wheels.
   */
  private double encTicksToMeters(final double et) {
    return et / ENC_TICKS_PER_METER;
  }

  // Updates the PID constants stored in each Talon FX from MoPrefs
  private void updatePIDConstants() {
    double kP = MoPrefs.getDriveKP();
    double kI = MoPrefs.getDriveKI();
    double kD = MoPrefs.getDriveKD();
    int kIZ = (int) MoPrefs.getDriveKIZ();
    double kFF = MoPrefs.getDriveKFF();
    leftFront.config_kP(0, kP);
    leftFront.config_kI(0, kI);
    leftFront.config_kD(0, kD);
    leftFront.config_IntegralZone(0, kIZ);
    leftFront.config_kF(0, kFF);
    rightFront.config_kP(0, kP);
    rightFront.config_kI(0, kI);
    rightFront.config_kD(0, kD);
    rightFront.config_IntegralZone(0, kIZ);
    rightFront.config_kF(0, kFF);
  }

  @Override
  public void periodic() {
    leftDriveVelocity.setDouble(encTicksToMeters(leftFront.getSensorCollection().getIntegratedSensorVelocity()));
    rightDriveVelocity.setDouble(encTicksToMeters(rightFront.getSensorCollection().getIntegratedSensorVelocity()));
    updatePIDConstants();
  }
}
