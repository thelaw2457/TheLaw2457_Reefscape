// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private final SparkMax m_elevatorMotorL = new SparkMax(47, SparkMax.MotorType.kBrushless);
  private final SparkMax m_elevatorMotorR = new SparkMax(42, SparkMax.MotorType.kBrushless);

  private final AnalogPotentiometer elevatorPot = new AnalogPotentiometer(3, 27, 0); //TODO: Change offset of pot

  public ElevatorSubsystem() {

    m_elevatorMotorL.configure(Configs.SparkConfigs.L_ELEV_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotorR.configure(Configs.SparkConfigs.R_ELEV_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Lift Pot.", elevatorPot.get());

  }

  public void set(double elevSpeed) {
    m_elevatorMotorL.set(elevSpeed);
    m_elevatorMotorR.set(elevSpeed);
  }

  public void stop() {
    m_elevatorMotorL.stopMotor();
    m_elevatorMotorR.stopMotor();
  }

  public double getPosition() {
    return elevatorPot.get();
  }

}
