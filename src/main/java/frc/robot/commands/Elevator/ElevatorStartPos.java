// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Mechanisms.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorStartPos extends Command {

  private ElevatorSubsystem ELEV_SUBSYSTEM;
  private PIDController pidController;

  /** Creates a new ElevatorLift. */
  public ElevatorStartPos(ElevatorSubsystem elev, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.ELEV_SUBSYSTEM = elev;

    this.pidController = new PIDController(Constants.ElevatorConstants.ELEV_KP, Constants.ElevatorConstants.ELEV_KI, Constants.ElevatorConstants.ELEV_KD);

    pidController.setSetpoint(setpoint);

    addRequirements(ELEV_SUBSYSTEM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(ELEV_SUBSYSTEM.getPosition());
    ELEV_SUBSYSTEM.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ELEV_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
