// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanisms.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorUp extends Command {

  private ElevatorSubsystem ELEV_SUBSYSTEM;
  private double elevSpeed;

  /** Creates a new ElevatorUp. */
  public ElevatorUp(ElevatorSubsystem elev, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.ELEV_SUBSYSTEM = elev;

    this.elevSpeed = speed;

    addRequirements(ELEV_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ELEV_SUBSYSTEM.set(elevSpeed);
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
