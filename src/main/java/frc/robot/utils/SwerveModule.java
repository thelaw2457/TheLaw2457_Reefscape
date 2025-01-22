package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveModule {
  public final int moduleNumber;

  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkClosedLoopController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;

  private final SparkMax angleMotor;
  private final RelativeEncoder angleEncoder;
  private final SparkClosedLoopController anglePID;
  
  private final CANCoder canCoder;
  private final double canCoderOffsetDegrees;

  private double lastAngle;

  public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
    this.moduleNumber = moduleNumber;
    
    driveMotor = new SparkMax(constants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getClosedLoopController();
    driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    angleMotor = new SparkMax(constants.angleMotorID, MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();
    anglePID = angleMotor.getClosedLoopController();

    canCoder = new CANCoder(constants.canCoderID);
    canCoderOffsetDegrees = constants.canCoderOffsetDegrees;

    configureDevices();
    lastAngle = getState().angle.getRadians();
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    // Prevents angle motor from turning further than it needs to. 
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    state = SwerveModuleState.optimize(state,  getState().angle);

    if (isOpenLoop) {
      double speed = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      drivePID.setReference(speed, SparkMax.ControlType.kDutyCycle);
    } else {
      drivePID.setReference(state.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    }

    double angle = Math.abs(state.speedMetersPerSecond) <= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
      ? lastAngle
      : state.angle.getRadians();

    anglePID.setReference(angle, SparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

  public double getCanCoder() {
    return canCoder.getAbsolutePosition();
  }

  public Rotation2d getAngle() {
    return new Rotation2d(angleEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  private void configureDevices() {
    // Drive motor configuration.
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
            .inverted(Constants.kSwerve.DRIVE_MOTOR_INVERSION)
            .idleMode(Constants.kSwerve.DRIVE_IDLE_MODE);
    driveConfig.encoder
            .positionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS)
            .velocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
    driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.kSwerve.DRIVE_KP, Constants.kSwerve.DRIVE_KI, Constants.kSwerve.DRIVE_KD);
 
    driveEncoder.setPosition(0);

    // Angle motor configuration.
    SparkMaxConfig angleConfig = new SparkMaxConfig();
    angleConfig
            .inverted(Constants.kSwerve.ANGLE_MOTOR_INVERSION)
            .idleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
    angleConfig.encoder
            .positionConversionFactor(Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS)
            .velocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
    angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.kSwerve.ANGLE_KP, Constants.kSwerve.ANGLE_KI, Constants.kSwerve.ANGLE_KD)
            .positionWrappingEnabled(true)
            .positionWrappingMaxInput(2 * Math.PI)
            .positionWrappingMinInput(0);

    angleEncoder.setPosition(Units.degreesToRadians(canCoder.getAbsolutePosition() - canCoderOffsetDegrees));

    // CanCoder configuration.
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.sensorDirection = Constants.kSwerve.CANCODER_INVERSION;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    
    canCoder.configFactoryDefault();
    canCoder.configAllSettings(canCoderConfiguration);
  }
}