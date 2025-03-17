// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanisms.FunnelPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotManualDown extends Command {

  private FunnelPivotSubsystem PIVOT_SUBSYSTEM;
  private double pivotSpeed;

  /** Creates a new PivotManualDown. */
  public PivotManualDown(FunnelPivotSubsystem pivot, double speed) {

    this.PIVOT_SUBSYSTEM = pivot;

    this.pivotSpeed = speed;

    addRequirements(PIVOT_SUBSYSTEM);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PIVOT_SUBSYSTEM.set(pivotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PIVOT_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
