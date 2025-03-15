// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanisms.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeSlurp extends Command {

  private AlgaeSubsystem ALGAE_SUBSYSTEM;
  private double algaeSpeed;

  /** Creates a new AlgaeSlurp. */
  public AlgaeSlurp(AlgaeSubsystem algae, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.ALGAE_SUBSYSTEM = algae;

    this.algaeSpeed = speed;

    addRequirements(ALGAE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ALGAE_SUBSYSTEM.set(algaeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ALGAE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
