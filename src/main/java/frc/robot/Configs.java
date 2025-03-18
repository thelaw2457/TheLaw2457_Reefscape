package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {
    
    public static final class SparkConfigs {
        public static final SparkMaxConfig FUNNEL_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig L_ELEV_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig R_ELEV_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig PIVOT_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig ALGAE_CONFIG = new SparkMaxConfig();

        static {
            FUNNEL_CONFIG.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

            L_ELEV_CONFIG.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
            R_ELEV_CONFIG.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

            PIVOT_CONFIG.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

            ALGAE_CONFIG.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
        }

    }

}
