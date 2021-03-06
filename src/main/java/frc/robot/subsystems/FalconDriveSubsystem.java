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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class FalconDriveSubsystem extends DriveSubsystem {
  // Need to use WPI_TalonFX so that DifferentialDrive will accept the motors.
  private final WPI_TalonFX leftFront = new WPI_TalonFX(FALCON_DRIVE_LEFT_FRONT_CAN_ADDR);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(FALCON_DRIVE_LEFT_REAR_CAN_ADDR);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(FALCON_DRIVE_RIGHT_FRONT_CAN_ADDR);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(FALCON_DRIVE_RIGHT_REAR_CAN_ADDR);

  private final SimEncoder leftSimEncoder = new SimEncoder();
  private final SimEncoder rightSimEncoder = new SimEncoder();

  private final ProfiledPIDController leftPID = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController rightPID = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(0, 0));

  private final DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2),
      GEAR_RATIO, 7 /* FIXME Get real moment of inertia */, 55, 0.1524, 0.5, null);

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

  private static final double DRIVE_BASE_WIDTH_METERS = Units.inchesToMeters(DRIVE_BASE_WIDTH_INCHES);

  private static final double TALON_FX_ENC_TICKS_PER_ROTATION = 2048;

  /**
   * The measured maximum velocity of the drivetrain, in encoder ticks per 100ms
   * FIXME Take a measurement for this
   */
  private static final double EMPIRICAL_MAX_VEL = 1891.2;

  private Field2d virtualField = new Field2d();

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(DRIVE_BASE_WIDTH_INCHES));

  private NetworkTableEntry leftDriveVelocity;
  private NetworkTableEntry rightDriveVelocity;

  private class SimEncoder {
    private double value;
    private double lastValue;
    private double lastTime = Timer.getFPGATimestamp();

    public SimEncoder() {
      value = 0;
      lastValue = 0;
      lastTime = 0;
    }

    public double get() {
      return value;
    }

    public double getVelocity() {
      return (value - lastValue) / (Timer.getFPGATimestamp() - lastTime);
    }

    /**
     * Should be called periodically during simulation.
     * 
     * @param newValue The new encoder value to set.
     */
    public void update(double newValue) {
      lastValue = value;
      lastTime = Timer.getFPGATimestamp();
      value = newValue;
    }
  }

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
      set(Utils.clip(moveRequest - turnRequest, -1, 1), Utils.clip(moveRequest + turnRequest, -1, 1));
    }
    leftFront.set(ControlMode.PercentOutput, Utils.clip(moveRequest - turnRequest, -1, 1));
    rightFront.set(ControlMode.PercentOutput, Utils.clip(moveRequest + turnRequest, -1, 1));
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
    // Scales each side speed based on the speed limit, then makes sure it's within
    // -1 and 1.
    // This is done the way it is to prevent errors if the target is faster than the
    // speed limit.
    double leftTarget = Utils.clip(wheelSpeeds.leftMetersPerSecond / SPEED_LIMIT_METERS_PER_S, -1, 1);
    double rightTarget = Utils.clip(wheelSpeeds.rightMetersPerSecond / SPEED_LIMIT_METERS_PER_S, -1, 1);
    // Sets the motors, with PIDF, to the setpoints.
    set(leftTarget, rightTarget);
  }

  /**
   * @param left  Left side target from -1 to 1
   * @param right Right side target from -1 to 1
   */
  private void set(double left, double right) {
    // Calculates PID output, inputs being:
    // 1 - The measured encoder velocity scaled to be (roughly) between -1 and 1.
    // 2 - The setpoint, between -1 and 1.
    leftFront.set(ControlMode.PercentOutput,
        leftPID.calculate(getEncoderVelocity(Side.LEFT) / EMPIRICAL_MAX_VEL, left));
    rightFront.set(ControlMode.PercentOutput,
        rightPID.calculate(getEncoderVelocity(Side.RIGHT) / EMPIRICAL_MAX_VEL, right));
  }

  public enum Side {
    LEFT, RIGHT
  }

  private double getEncoderVelocity(Side side) {
    if (RobotBase.isReal()) {
      return side == Side.LEFT ? leftFront.getSensorCollection().getIntegratedSensorVelocity()
          : rightFront.getSensorCollection().getIntegratedSensorVelocity();
    } else {
      return side == Side.LEFT ? leftSimEncoder.getVelocity() : rightSimEncoder.getVelocity();
    }
  }

  private double getEncoderDistance(Side side) {
    if (RobotBase.isReal()) {
      // TODO: getIntegratedSensorAbsolutePosition or getIntegratedSensorPosition?
      return side == Side.LEFT ? leftFront.getSensorCollection().getIntegratedSensorAbsolutePosition()
          : rightFront.getSensorCollection().getIntegratedSensorAbsolutePosition();
    } else {
      return side == Side.LEFT ? leftSimEncoder.get() : rightSimEncoder.get();
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
   * @return encoder ticks
   */
  private double metersToEncTicks(final double m) {
    return m * ENC_TICKS_PER_METER;
  }

  /**
   * 
   * @param et The number of encoder ticks to be converted to meters.
   * @return meters
   */
  private double encTicksToMeters(final double et) {
    return et / ENC_TICKS_PER_METER;
  }

  // Updates the PID constants stored in the PID controllers from MoPrefs
  private void updatePIDConstants() {
    double kP = MoPrefs.getInstance().get(MoPrefsKey.DRIVE_KP);
    double kI = MoPrefs.getInstance().get(MoPrefsKey.DRIVE_KI);
    double kD = MoPrefs.getInstance().get(MoPrefsKey.DRIVE_KD);
    int kIZ = (int) MoPrefs.getInstance().get(MoPrefsKey.DRIVE_KIZ);
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    leftPID.setIntegratorRange(-kIZ, kIZ);
    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
    leftPID.setIntegratorRange(-kIZ, kIZ);
  }

  public Pose2d getPose() {
    return generatePose(getEncoderDistance(Side.LEFT), getEncoderDistance(Side.RIGHT));
  }

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

  /**
   * 
   * @param leftVel  Left side encoder velocity, in Talon FX encoder ticks per
   *                 100ms
   * @param rightVel Right side encoder velocity, in Talon FX encoder ticks per
   *                 100ms
   * @return a Pose2d representing the motion of the robot
   */
  public Pose2d generatePose(double leftDist, double rightDist) {
    double rot = (leftDist - rightDist) * 2 /* Rotations to radians CF, since pi cancels */
        / (DRIVE_BASE_WIDTH_METERS);

    odometry.update(new Rotation2d(rot), leftDist, rightDist);

    return odometry.getPoseMeters();
  }

  /**
   * @param encoderVelocity Input velocity in Talon FX encoder ticks per 100ms
   * @return Velocity of the drive wheels in meters per second
   */
  public double getWheelVelocity(double encoderVelocity) {
    double encTicksPerSecond = encoderVelocity * 10; // Talon FX outputs in encoder ticks per 100ms, but we want it per
                                                     // second
    double rotationsPerSecond = encTicksPerSecond / TALON_FX_ENC_TICKS_PER_ROTATION; // 2048 encoder ticks = 1 rotation

    return rotationsPerSecond * GEAR_RATIO * WHEEL_DIAMETER * Math.PI;
  }

  public void resetOdo() {
    odometry.resetPosition(new Pose2d(), new Rotation2d());
    leftFront.getSensorCollection().setIntegratedSensorPosition(0, 0);
    rightFront.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  @Override
  public void periodic() {
    leftDriveVelocity.setDouble(getEncoderVelocity(Side.LEFT));
    rightDriveVelocity.setDouble(getEncoderVelocity(Side.RIGHT));
    updatePIDConstants();
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs(Utils.map(leftFront.get(), -1, 1, -12, 12), Utils.map(rightFront.get(), -1, 1, -12, 12));
    drivetrainSim.update(0.020); // 20ms, the recommended period

    leftSimEncoder.update(metersToEncTicks(drivetrainSim.getLeftPositionMeters()));
    rightSimEncoder.update(metersToEncTicks(drivetrainSim.getRightPositionMeters()));

    SmartDashboard.putNumber("left drive setting", leftFront.get());
    SmartDashboard.putNumber("right drive setting", rightFront.get());
    SmartDashboard.putNumber("DTSim left pos m", drivetrainSim.getLeftPositionMeters());
    SmartDashboard.putNumber("DTSim left vel m per s", drivetrainSim.getLeftVelocityMetersPerSecond());
    // Adds a field image to the simulation GUI which helps visualize simulated
    // autonomous routines.

    virtualField.setRobotPose(getPose());
    SmartDashboard.putData(virtualField);
    SmartDashboard.putNumber("left encoder val", leftSimEncoder.get());
    SmartDashboard.putNumber("right encoder val", rightSimEncoder.get());
  }
}
