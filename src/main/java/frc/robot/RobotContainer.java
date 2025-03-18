// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Algae.AlgaeSlurp;
import frc.robot.commands.Coral.funnelSpit;
import frc.robot.commands.Elevator.ElevatorClimb;
import frc.robot.commands.Elevator.ElevatorLvl1;
import frc.robot.commands.Elevator.ElevatorLvl2;
import frc.robot.commands.Elevator.ElevatorLvl3;
import frc.robot.commands.Elevator.ElevatorStartPos;
import frc.robot.commands.Pivot.PIvotIntakePos;
import frc.robot.commands.Pivot.PivotHighPos;
import frc.robot.commands.Pivot.PivotMidPos;
import frc.robot.commands.Elevator.ElevatorUp;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Pivot.PivotManualUp;
import frc.robot.commands.Pivot.PivotManualDown;
import frc.robot.subsystems.Mechanisms.AlgaeSubsystem;
import frc.robot.subsystems.Mechanisms.CoralFunnel;
import frc.robot.subsystems.Mechanisms.ElevatorSubsystem;
import frc.robot.subsystems.Mechanisms.FunnelPivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // final         Joystick              driverJoystick = new Joystick(1);
  final XboxController operatorXbox = new XboxController(1);
  final XboxController secondXbox = new XboxController(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final FunnelPivotSubsystem m_funnelPivotSubsystem = new FunnelPivotSubsystem();
  private final CoralFunnel m_coralFunnelSubsystem = new CoralFunnel();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(null);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driveDirectAngleKeyboard.driveToPose(() -> new Pose2d(new Translation2d(9, 3),
                                                            Rotation2d.fromDegrees(90)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5,
                                                                                     3)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(
                                                                         Math.toRadians(
                                                                             360),
                                                                         Math.toRadians(
                                                                             90))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  private void configureButtonBindings()
  {
    // Configure your button bindings here

    //Elevator Control
    new POVButton(operatorXbox, 180).whileTrue(new ElevatorStartPos(m_elevatorSubsystem, Constants.ElevatorConstants.ELEV_START));
    new JoystickButton(secondXbox, 3).whileTrue(new ElevatorLvl1(m_elevatorSubsystem, Constants.ElevatorConstants.ELEV_LVL1));
    new POVButton(operatorXbox, 90).whileTrue(new ElevatorLvl2(m_elevatorSubsystem, Constants.ElevatorConstants.ELEV_LVL2));
    new POVButton(operatorXbox, 0).whileTrue(new ElevatorLvl3(m_elevatorSubsystem, Constants.ElevatorConstants.ELEV_LVL3));
    new JoystickButton(operatorXbox, 4).whileTrue(new ElevatorClimb(m_elevatorSubsystem, Constants.ElevatorConstants.ELEV_CLIMB));

    //Funnel Pivot Control
    new JoystickButton(operatorXbox, 3).whileTrue(new PIvotIntakePos(m_funnelPivotSubsystem, Constants.PivotConstants.PIVOT_INTAKE));
    new JoystickButton(operatorXbox, 1).whileTrue(new PivotMidPos(m_funnelPivotSubsystem, Constants.PivotConstants.PIVOT_MID_SCORE));
    new JoystickButton(operatorXbox, 2).whileTrue(new PivotHighPos(m_funnelPivotSubsystem, Constants.PivotConstants.PIVOT_HIGH_SCORE));

    new JoystickButton(secondXbox, 5).whileTrue(new PivotManualUp(m_funnelPivotSubsystem, Constants.PivotConstants.PIVOT_UP_SPEED));
    new JoystickButton(secondXbox, 6).whileTrue(new PivotManualDown(m_funnelPivotSubsystem, Constants.PivotConstants.PIVOT_DOWN_SPEED));

    //Coral Spit & Algae Slurp
    new JoystickButton(operatorXbox, 6).whileTrue(new funnelSpit(m_coralFunnelSubsystem, Constants.FunnelConstants.FUNNEL_SPIT_SPEED));
    new JoystickButton(operatorXbox, 5).whileTrue(new AlgaeSlurp(m_algaeSubsystem, Constants.AlgaeConstants.ALGAE_SLURP_SPEED));

    //Manual Up & Down
    new JoystickButton(secondXbox, 1).whileTrue(new ElevatorUp(m_elevatorSubsystem, Constants.ElevatorConstants.ELEV_UP));
    new JoystickButton(secondXbox, 2).whileTrue(new ElevatorDown(m_elevatorSubsystem, Constants.ElevatorConstants.ELEV_DOWN));

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
