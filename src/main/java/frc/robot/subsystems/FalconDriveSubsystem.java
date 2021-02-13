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
import frc.robot.utils.MoPrefs.MoPrefsKey;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
   * wheels. Used to input m/s into WPI_TalonFX.set(VelocityControl, double
   * value), which takes its value in encoder ticks per 100ms.
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

  private static final double GEAR_RATIO = 1 / 10.75;

  private static final double WHEEL_DIAMETER = 0.1524; // meters

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(DRIVE_BASE_WIDTH_INCHES));

  private NetworkTableEntry leftDriveVelocity;
  private NetworkTableEntry rightDriveVelocity;

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
    // Sets the acceleration limit for all motors.
    leftFront.configMotionAcceleration(ACCELERATION_LIMIT);
    rightFront.configMotionAcceleration(ACCELERATION_LIMIT);

    if (tab != null) {
      NetworkTableEntry drivePIDchooser = tab.add("Drive PID", false).withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();
      drivePIDchooser.addListener(notice -> enablePID = notice.value.getBoolean(),
          EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      leftDriveVelocity = tab.add("Drive Velocity L", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
      rightDriveVelocity = tab.add("Drive Velocity R", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    }
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

  /**
   * Drives the robot, according to a ChassisSpeeds object from a
   * RamseteController.
   * 
   * @param chassisSpeeds The desired forward/backward and rotational movement, in
   *                      m/s and rad/s, respectively.
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    // Converts chassis speeds (the whole robot) into per-side speeds.
    final DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    // Converts the individual side speeds in m/s to encoder ticks per second
    double leftETPerS = metersToEncTicks(wheelSpeeds.leftMetersPerSecond);
    double rightETPerS = metersToEncTicks(wheelSpeeds.rightMetersPerSecond);
    // Divides the side speeds by 10 to convert them to encoder ticks per 100ms
    leftETPerS *= 0.1;
    rightETPerS *= 0.1;
    // Feeds the encoder ticks per second setpoint into the Talon FX PID
    // controllers.
    leftFront.set(ControlMode.MotionMagic, leftETPerS);
    rightFront.set(ControlMode.MotionMagic, rightETPerS);
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
   * @return The number of meters that correspond to @param et , assuming 6-inch
   *         wheels.
   */
  private double encTicksToMeters(final double et) {
    return et / ENC_TICKS_PER_METER;
  }

  // Updates the PID constants stored in each Talon FX from MoPrefs
  private void updatePIDConstants() {
    double kP = MoPrefs.get(MoPrefsKey.DRIVE_KP);
    double kI = MoPrefs.get(MoPrefsKey.DRIVE_KI);
    double kD = MoPrefs.get(MoPrefsKey.DRIVE_KD);
    int kIZ = (int) MoPrefs.get(MoPrefsKey.DRIVE_KIZ);
    double kFF = MoPrefs.get(MoPrefsKey.DRIVE_KFF);
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

  public Pose2d getPose() {
    return generatePose(getWheelVelocity(leftFront.getSensorCollection().getIntegratedSensorVelocity()),
        getWheelVelocity(rightFront.getSensorCollection().getIntegratedSensorVelocity()));
  }

  /**
   * 
   * @param leftVel  Left side encoder velocity, in Talon FX encoder ticks per
   *                 100ms
   * @param rightVel Right side encoder velocity, in Talon FX encoder ticks per
   *                 100ms
   * @return a Pose2d representing the motion of the robot
   */
  public Pose2d generatePose(double leftVel, double rightVel) {
    leftVel = getWheelVelocity(leftVel);
    rightVel = getWheelVelocity(rightVel);
    double x = (leftVel + rightVel) / 2; // Averages the two velocities to get the robot velocity
    double y = 0; // Not a holonomic drive
    double rot = (leftVel - rightVel) * 2 /* Rotations to radians CF, since pi cancels */
        / (DRIVE_BASE_WIDTH_INCHES * 0.0254/* inch to meter */);
    return new Pose2d(x, y, new Rotation2d(rot));
  }

  /**
   * @param encoderVelocity Input velocity in Talon FX encoder ticks per 100ms
   * @return Velocity of the drive wheels in meters per second
   */
  public double getWheelVelocity(double encoderVelocity) {
    double encTicksPerSecond = encoderVelocity * 10; // Talon FX outputs in encoder ticks per 100ms, but we want it per
                                                     // second
    double rotationsPerSecond = encTicksPerSecond / 2048; // 2048 encoder ticks = 1 rotation

    return rotationsPerSecond * GEAR_RATIO * WHEEL_DIAMETER * Math.PI;
  }

  @Override
  public void periodic() {
    leftDriveVelocity.setDouble(encTicksToMeters(leftFront.getSensorCollection().getIntegratedSensorVelocity()));
    rightDriveVelocity.setDouble(encTicksToMeters(rightFront.getSensorCollection().getIntegratedSensorVelocity()));
    updatePIDConstants();
  }
}
