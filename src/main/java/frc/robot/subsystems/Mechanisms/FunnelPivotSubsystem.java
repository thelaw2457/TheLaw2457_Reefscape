// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class FunnelPivotSubsystem extends SubsystemBase {

  private final SparkMax m_funnelPivotMotor = new SparkMax(37, SparkMax.MotorType.kBrushless);

  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(9);

  /** Creates a new FunnelPivotSubsystem. */
  public FunnelPivotSubsystem() {

    m_funnelPivotMotor.configure(Configs.SparkConfigs.PIVOT_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Funnel Pivot Encoder", pivotEncoder.get());

  }

  public void set(double pivotSpeed) {
    m_funnelPivotMotor.set(pivotSpeed);
  }

  public void stop() {
    m_funnelPivotMotor.stopMotor();
  }

  public double getPosition() {
    return pivotEncoder.get();
  }

}
