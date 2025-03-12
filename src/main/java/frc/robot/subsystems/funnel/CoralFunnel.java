package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Configs;

public class CoralFunnel {
    
    private final SparkMax m_funnelWheel = new SparkMax(30, MotorType.kBrushless);

    public CoralFunnel() {

        m_funnelWheel.configure(Configs.CoralFunnel.FUNNEL_CONFIG, null, null);

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
