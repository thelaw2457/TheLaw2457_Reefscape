package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {
    
    public static final class CoralFunnel {
        public static final SparkMaxConfig FUNNEL_CONFIG = new SparkMaxConfig();

        static {
            FUNNEL_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
        }

    }

}
