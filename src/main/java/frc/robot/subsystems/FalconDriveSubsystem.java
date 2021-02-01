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
  /**
   * The Proportional Gain, used in PID to ramp velocity in relation to error.
   */
  private double kP = 0;
  /**
   * The Integral Gain, used in PID to correct steady-state error and combat
   * friction.
   */
  private double kI = 0;
  /**
   * The Differential Gain, used in PID to dampen the output of the PID controller
   * to reduce oscillations.
   */
  private double kD = 0;
  /**
   * The Integral Zone, used in PID to control the maximum value of the integral
   * accumulator.
   */
  private int kIZ = 0;
  /**
   * The Feed-Forward Gain, used in PID to anticipate future changes in error and
   * stabilize a PID curve.
   */
  private double kFF = 1;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(DRIVE_BASE_WIDTH_INCHES));

  private NetworkTableEntry leftDriveVelocity;
  private NetworkTableEntry rightDriveVelocity;
  private NetworkTableEntry kPSlider;
  private NetworkTableEntry kISlider;
  private NetworkTableEntry kIZSlider;
  private NetworkTableEntry kDSlider;
  private NetworkTableEntry kFFSlider;

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

    leftDriveVelocity = tab.add("Drive Velocity (L), m/s", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    rightDriveVelocity = tab.add("Drive Velocity (R), m/s", 0).withWidget(BuiltInWidgets.kGraph).getEntry();

    kPSlider = tab.addPersistent("kP", 5e-5).withWidget(BuiltInWidgets.kTextView).getEntry();
    kPSlider.addListener(notice -> kP = notice.value.getDouble(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kISlider = tab.addPersistent("kI", 1e-6).withWidget(BuiltInWidgets.kTextView).getEntry();
    kISlider.addListener(notice -> kI = notice.value.getDouble(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kIZSlider = tab.addPersistent("kIZ", 1e-6).withWidget(BuiltInWidgets.kTextView).getEntry();
    kIZSlider.addListener(notice -> kIZ = (int) notice.value.getDouble(),
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kDSlider = tab.addPersistent("kD", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    kDSlider.addListener(notice -> kD = notice.value.getDouble(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kFFSlider = tab.addPersistent("kFF", 0.000156).withWidget(BuiltInWidgets.kTextView).getEntry();
    kFFSlider.addListener(notice -> kFF = notice.value.getDouble(),
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
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
      final double moveReqScaled = Utils.map(moveRequest, -1, 1, -SPEED_LIMIT_METERS_PER_S, SPEED_LIMIT_METERS_PER_S);
      /**
       * Since turnRequest is from -1 to 1 and we need a value in radians per second
       * to feed to ChassisSpeeds, we scale the turnRequest to the angular velocity
       * limit in radians/second.
       */
      final double turnReqScaled = Utils.map(turnRequest, -1, 1, -TURN_LIMIT_RAD_PER_S, TURN_LIMIT_RAD_PER_S);
      /**
       * Calls the overload of this method, since all conversion is done.
       */
      drive(new ChassisSpeeds(moveReqScaled, 0, turnReqScaled));
    } else {
      leftFront.set(ControlMode.PercentOutput, Utils.clip(moveRequest - turnRequest, -1, 1));
      rightFront.set(ControlMode.PercentOutput, Utils.clip(moveRequest + turnRequest, -1, 1));
    }
  }

  private double[] chassisSpeedsToEncoderTicks(ChassisSpeeds chassisSpeeds) {
    final DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    final double leftMPerS = wheelSpeeds.leftMetersPerSecond;
    final double rightMPerS = wheelSpeeds.rightMetersPerSecond;
    final double leftETPerS = metersToEncTicks(leftMPerS);
    final double rightETPerS = metersToEncTicks(rightMPerS);
    return new double[] { leftETPerS, rightETPerS };
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    double[] wheelSpeeds = chassisSpeedsToEncoderTicks(chassisSpeeds);
    leftFront.set(ControlMode.MotionMagic, wheelSpeeds[0]);
    rightFront.set(ControlMode.MotionMagic, wheelSpeeds[1]);
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

  // Since the Shuffleboard widgets have listeners, all the PID constants are
  // updated in the original variables.
  private void updatePIDConstants() {
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
