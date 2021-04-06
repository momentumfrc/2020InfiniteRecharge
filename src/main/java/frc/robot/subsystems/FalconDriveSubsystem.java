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
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.SimGyro;
import frc.robot.utils.MoPrefs.MoPrefsKey;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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

public class FalconDriveSubsystem extends DriveSubsystem {
  // Need to use WPI_TalonFX so that DifferentialDrive will accept the motors.
  private final WPI_TalonFX leftFront = new WPI_TalonFX(FALCON_DRIVE_LEFT_FRONT_CAN_ADDR);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(FALCON_DRIVE_LEFT_REAR_CAN_ADDR);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(FALCON_DRIVE_RIGHT_FRONT_CAN_ADDR);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(FALCON_DRIVE_RIGHT_REAR_CAN_ADDR);

  private final SimEncoder leftSimEncoder = new SimEncoder();
  private final SimEncoder rightSimEncoder = new SimEncoder();

  private final SimGyro simGyro = new SimGyro();

  private final Gyro gyro;

  private final PIDController leftPID = new PIDController(0, 0, 0);
  private final PIDController rightPID = new PIDController(0, 0, 0);

  private final DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(Constants.kDrivetrainPlant,
      DCMotor.getFalcon500(2), GEAR_RATIO, 0.1524, 0.0762, null);

  private boolean enablePID = false;

  private static final double WHEEL_DIAMETER = 0.1524; // meters

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

  private static final double DRIVE_BASE_WIDTH_METERS = Units.inchesToMeters(DRIVE_BASE_WIDTH_INCHES);

  private static final double TALON_FX_ENC_TICKS_PER_ROTATION = 2048;

  /**
   * The number of TalonFX encoder ticks per meter that the robot drives on 6"
   * wheels. Used to input m/s into WPI_TalonFX.set(VelocityControl, double
   * value), which takes its value in encoder ticks per 100ms.
   */
  private static final double ENC_TICKS_PER_METER = (1 / (Math.PI * WHEEL_DIAMETER)) * (1 / GEAR_RATIO)
      * TALON_FX_ENC_TICKS_PER_ROTATION;

  /**
   * The measured maximum velocity of the drivetrain, in encoder ticks per 100ms
   */
  private static final double EMPIRICAL_MAX_VEL = 1891.2;

  private Field2d virtualField = new Field2d();

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(DRIVE_BASE_WIDTH_INCHES));

  private NetworkTableEntry leftDriveVelocity;
  private NetworkTableEntry rightDriveVelocity;

  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

  private boolean isReal;

  /**
   * Can technically be used with any units, but the FalconDriveSubsystem usecase
   * expects everything to be in Talon FX encoder ticks per 100ms
   */
  private class SimEncoder {
    private double position; // ticks
    private double velocity; // ticks per decisecond

    public SimEncoder() {
      position = 0;
      velocity = 0;
    }

    public double get() {
      return position;
    }

    public double getVelocity() {
      return velocity;
    }

    /**
     * Should be called periodically during simulation.
     * 
     * @param newValue The new encoder value to set.
     */
    public void update(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }
  }

  public FalconDriveSubsystem(ShuffleboardTab tab, AHRS gyro) {
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

    // leftPID.enableContinuousInput(-1, 1);
    // rightPID.enableContinuousInput(-1, 1);

    this.gyro = gyro;

    isReal = RobotBase.isReal();

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
  @Override
  public void drive(final double moveRequest, final double turnRequest) {
    double left = Utils.clip(moveRequest - turnRequest, -1, 1);
    double right = Utils.clip(moveRequest + turnRequest, -1, 1);
    if (enablePID) {
      set(Utils.map(left, -1, 1, -SPEED_LIMIT_METERS_PER_S, SPEED_LIMIT_METERS_PER_S),
          Utils.map(right, -1, 1, -SPEED_LIMIT_METERS_PER_S, SPEED_LIMIT_METERS_PER_S));
    } else {
      leftFront.set(ControlMode.PercentOutput, left);
      rightFront.set(ControlMode.PercentOutput, right);
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
    // Clips each side to the speed limit.
    double leftTarget = Utils.clip(wheelSpeeds.leftMetersPerSecond, -SPEED_LIMIT_METERS_PER_S,
        SPEED_LIMIT_METERS_PER_S);
    double rightTarget = Utils.clip(wheelSpeeds.rightMetersPerSecond, -SPEED_LIMIT_METERS_PER_S,
        SPEED_LIMIT_METERS_PER_S);

    // Sets the motors, with PID, to the setpoints.
    set(leftTarget, rightTarget);
  }

  /**
   * Drives the robot using PIDF.
   * 
   * @param left  Left side target in m/s
   * @param right Right side target in m/s
   */
  private void set(double left, double right) {
    SmartDashboard.putNumber("left PID target", left);
    SmartDashboard.putNumber("right PID target", right);

    // Calculates PID output, inputs being:
    // 1 - The measured encoder velocity converted to wheel speeds in m/s.
    // 2 - The setpoint in wheel speeds, m/s.
    double leftMeasurement = getWheelVelocity(getEncoderVelocity(Side.LEFT));
    double rightMeasurement = -getWheelVelocity(getEncoderVelocity(Side.RIGHT));

    double leftOutput = leftPID.calculate(leftMeasurement, left);
    double rightOutput = rightPID.calculate(rightMeasurement, right);

    SmartDashboard.putNumber("left PID output", leftOutput);
    SmartDashboard.putNumber("right PID output", rightOutput);

    double kFF = 1 / SPEED_LIMIT_METERS_PER_S;

    leftOutput += kFF * left;
    rightOutput += kFF * right;

    // leftFront.set(ControlMode.PercentOutput, left / 30);
    // rightFront.set(ControlMode.PercentOutput, right / 30);

    leftFront.set(ControlMode.PercentOutput, leftOutput);
    rightFront.set(ControlMode.PercentOutput, rightOutput);

    SmartDashboard.putNumber("left measurement", leftMeasurement);
    SmartDashboard.putNumber("right measurement", rightMeasurement);
    SmartDashboard.putNumber("left error", leftPID.getPositionError());
    SmartDashboard.putNumber("right err", rightPID.getPositionError());
  }

  public enum Side {
    LEFT, RIGHT
  }

  private double getEncoderVelocity(Side side) {
    if (isReal) {
      return side == Side.LEFT ? leftFront.getSensorCollection().getIntegratedSensorVelocity()
          : -rightFront.getSensorCollection().getIntegratedSensorVelocity();
    } else {
      return side == Side.LEFT ? leftSimEncoder.getVelocity() : -rightSimEncoder.getVelocity();
    }
  }

  private double getEncoderDistance(Side side) {
    if (isReal) {
      // TODO: getIntegratedSensorAbsolutePosition or getIntegratedSensorPosition?
      return side == Side.LEFT ? leftFront.getSensorCollection().getIntegratedSensorAbsolutePosition()
          : -rightFront.getSensorCollection().getIntegratedSensorAbsolutePosition();
    } else {
      return side == Side.LEFT ? leftSimEncoder.get() : rightSimEncoder.get();
    }
  }

  // returns in radians
  private double getGyroAngle() {
    // FIXME: gyro.getAngle is continuous, but simGyro.getAngle() is not
    return isReal ? gyro.getAngle() : simGyro.getAngle();
  }

  private double getGyroRate() {
    return isReal ? gyro.getRate() : simGyro.getRate();
  }

  /**
   * Stops both motors.
   */
  public void stop() {
    leftFront.stopMotor();
    rightFront.stopMotor();
  }

  /**
   * @param m The number of meters to be converted to encoder ticks.
   * @return encoder ticks
   */
  private double metersToEncTicks(final double m) {
    return m * ENC_TICKS_PER_METER;
  }

  private double mpsToTicksPerDeciSecond(double m) {
    return (m * ENC_TICKS_PER_METER) / 10;
  }

  /**
   * 
   * @param et The number of encoder ticks to be converted to meters.
   * @return meters
   */
  @SuppressWarnings("unused")
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
    return generatePose();
  }

  /**
   * @return a Pose2d representing the motion of the robot
   */
  public Pose2d generatePose() {
    return odometry.getPoseMeters();
  }

  /**
   * @param encoderVelocity Encoder ticks per decisecond (100ms)
   * @return Wheel meters per second
   */
  public double getWheelVelocity(double encoderVelocity) {
    return (encoderVelocity / ENC_TICKS_PER_METER) * 10;
  }

  /**
   * @param wheelVelocity Wheel meters per second
   * @return Encoder ticks per decisecond (100ms)
   */
  public double convWheelToMotorVelocity(double wheelVelocity) {
    return (wheelVelocity * ENC_TICKS_PER_METER) / 10;
  }

  public void resetOdo(Pose2d newPose) {
    odometry.resetPosition(newPose, new Rotation2d());
    leftFront.getSensorCollection().setIntegratedSensorPosition(0, 0);
    rightFront.getSensorCollection().setIntegratedSensorPosition(0, 0);
    leftSimEncoder.update(0, 0);
    rightSimEncoder.update(0, 0);
    drivetrainSim.setPose(newPose);
    drivetrainSim.setInputs(0, 0);
    leftPID.reset();
    rightPID.reset();
  }

  public void resetOdo() {
    odometry.resetPosition(new Pose2d(), new Rotation2d());
    leftFront.getSensorCollection().setIntegratedSensorPosition(0, 0);
    rightFront.getSensorCollection().setIntegratedSensorPosition(0, 0);
    leftSimEncoder.update(0, 0);
    rightSimEncoder.update(0, 0);
    drivetrainSim.setPose(new Pose2d());
    drivetrainSim.setInputs(0, 0);
    leftPID.reset();
    rightPID.reset();
  }

  public PIDController getLeftPidController() {
    return leftPID;
  }

  public PIDController getRightPidController() {
    return rightPID;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encTicksToMeters(getEncoderVelocity(Side.LEFT)),
        encTicksToMeters(getEncoderVelocity(Side.RIGHT)));
  }

  public void tankDriveVolts(double left, double right) {
    set(left / RobotController.getBatteryVoltage(), right / RobotController.getBatteryVoltage());
  }

  @Override
  public void periodic() {
    leftDriveVelocity.setDouble(getEncoderVelocity(Side.LEFT));
    rightDriveVelocity.setDouble(getEncoderVelocity(Side.RIGHT));
    odometry.update(Rotation2d.fromDegrees(getGyroAngle()), encTicksToMeters(getEncoderDistance(Side.LEFT)),
        encTicksToMeters(getEncoderDistance(Side.RIGHT)));
    updatePIDConstants();
    Pose2d pose = generatePose();
    SmartDashboard.putNumber("pose x", pose.getX());
    SmartDashboard.putNumber("pose y", pose.getY());
    SmartDashboard.putNumber("pose rot", pose.getRotation().getRadians());
    SmartDashboard.putNumber("left enc val", encTicksToMeters(getEncoderDistance(Side.LEFT)));
    SmartDashboard.putNumber("right enc val", encTicksToMeters(getEncoderDistance(Side.RIGHT)));
  }

  @Override
  public void simulationPeriodic() {
    double currBatteryVoltage = RobotController.getBatteryVoltage();

    double left_pwr = leftFront.get();
    double right_pwr = -rightFront.get();
    double left_volt = left_pwr * currBatteryVoltage;
    double right_volt = right_pwr * currBatteryVoltage;
    drivetrainSim.setInputs(left_volt, right_volt);

    drivetrainSim.update(0.020); // 20ms, the recommended period

    leftSimEncoder.update(metersToEncTicks(drivetrainSim.getLeftPositionMeters()),
        convWheelToMotorVelocity(drivetrainSim.getLeftVelocityMetersPerSecond()));
    rightSimEncoder.update(metersToEncTicks(drivetrainSim.getRightPositionMeters()),
        convWheelToMotorVelocity(drivetrainSim.getRightVelocityMetersPerSecond()));
    simGyro.update(drivetrainSim.getHeading().getDegrees());

    SmartDashboard.putNumber("DTSim left pos m", drivetrainSim.getLeftPositionMeters());
    SmartDashboard.putNumber("DTSim right pos m", drivetrainSim.getRightPositionMeters());
    SmartDashboard.putNumber("DTSim left vel m per s", drivetrainSim.getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("DTSim right vel ms", drivetrainSim.getRightVelocityMetersPerSecond());
    // Adds a field image to the simulation GUI which helps visualize simulated
    // autonomous routines.

    virtualField.setRobotPose(getPose());
    SmartDashboard.putData(virtualField);
    SmartDashboard.putNumber("left encoder vel", leftSimEncoder.getVelocity());
    SmartDashboard.putNumber("left enc value", rightSimEncoder.get());
    SmartDashboard.putNumber("gyro angle", getGyroAngle());
  }
}
