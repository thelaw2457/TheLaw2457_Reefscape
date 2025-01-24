package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;
import com.studica.frc.AHRS;
import java.util.function.DoubleSupplier;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule[] modules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private AHRS gyro;
  
  private Field2d field = new Field2d();
  
  public SwerveSubsystem() {
    gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    zeroGyro();

    modules = new SwerveModule[] {
        new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
        new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
        new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
        new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
      };

      kinematics = Constants.kSwerve.KINEMATICS;
    // kinematics = new SwerveDriveKinematics(
    //   Constants.kSwerve.flModuleOffset, 
    //   Constants.kSwerve.frModuleOffset, 
    //   Constants.kSwerve.blModuleOffset, 
    //   Constants.kSwerve.brModuleOffset
    // );
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          Constants.kSwerve.translationConstants,
          Constants.kSwerve.rotationConstants
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);

    public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isFieldRelative, boolean isOpenLoop, double seconds) {
        return run(() -> {
    
          // Grabbing input from suppliers.
          double forwardBack = forwardBackAxis.getAsDouble()*8;
          double leftRight = leftRightAxis.getAsDouble()*8;
          double rotation = rotationAxis.getAsDouble()*10;
    
          // Adding deadzone.
          forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
          leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
          rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;
    
          // Get desired module states.
          ChassisSpeeds chassisSpeeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
            : new ChassisSpeeds(forwardBack, leftRight, rotation);
    
          SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    
          setModuleStates(states, isOpenLoop);
        }).withName("SwerveDriveCommand");
      }
    
      public void drive(final double forwardBack, final double leftRight, final double rotation) {
        SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw()));
        setModuleStates(states, true);
      }
    
      public double getPitch() {
        return gyro.getPitch();
      }

      public Command zeroGyroCommand() {
        return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
      }
    
      private void zeroGyro() {
        gyro.zeroYaw();
      }

  }

  @Override
  public void periodic() {
  
    public double getPitch() {
      return gyro.getPitch();
    }

    odometry.update(gyro.getRotation2d(), getPositions());

    field.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    System.out.println(pose);
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    // for (int i = 0; i < modules.length; i++) {
    //   modules[i].setTargetState(targetStates[i]);
    // }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  
  
}