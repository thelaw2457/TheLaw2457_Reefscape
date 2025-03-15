package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Configs;

public class CoralFunnel extends SubsystemBase {
    
    private final SparkMax m_funnelWheel = new SparkMax(30, MotorType.kBrushless);

    public CoralFunnel() {

        m_funnelWheel.configure(Configs.SparkConfigs.FUNNEL_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

public void periodic() {
    // This method will be called once per scheduler run
    
}

public void set(double wheelSpeed) {
    m_funnelWheel.set(wheelSpeed);
}

public void stop() {
    m_funnelWheel.stopMotor();
}

}
