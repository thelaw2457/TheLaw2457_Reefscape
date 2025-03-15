// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class AlgaeSubsystem extends SubsystemBase {

  private final SparkMax m_algaeMotor = new SparkMax(60, SparkMax.MotorType.kBrushless);

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {

    m_algaeMotor.configure(Configs.SparkConfigs.ALGAE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double algaeSpeed) {
    m_algaeMotor.set(algaeSpeed);
  }

  public void stop() {
    m_algaeMotor.stopMotor();
  }

}
