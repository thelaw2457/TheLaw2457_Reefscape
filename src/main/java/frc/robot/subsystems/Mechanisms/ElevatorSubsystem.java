// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private final SparkMax m_elevatorMotorL = new SparkMax(47, SparkMax.MotorType.kBrushless);
  private final SparkMax m_elevatorMotorR = new SparkMax(42, SparkMax.MotorType.kBrushless);

  private final AnalogPotentiometer elevatorPot = new AnalogPotentiometer(3, 27, -0.215); //TODO: Change offset of pot

  private final DigitalInput lvl2Switch = new DigitalInput(8);
  private final DigitalInput lvl3Switch = new DigitalInput(7);
  private final DigitalInput homeSwitch = new DigitalInput(5);

  public ElevatorSubsystem() {

    m_elevatorMotorL.configure(Configs.SparkConfigs.L_ELEV_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorMotorR.configure(Configs.SparkConfigs.R_ELEV_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Lift Pot.", elevatorPot.get());

    SmartDashboard.putBoolean("lvl 2 Switch", lvl2Switch.get());
    SmartDashboard.putBoolean("lvl 3 Switch", lvl3Switch.get());
    SmartDashboard.putBoolean("home Switch", homeSwitch.get());

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

  public boolean lvl2SwitchVal() {
    return lvl2Switch.get();
  }

  public boolean lvl3SwitchVal() {
    return lvl3Switch.get();
  }

  public boolean homeSwitchVal() {
    return homeSwitch.get();
  }

}
