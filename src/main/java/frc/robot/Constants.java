package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

public class Constants {
    public static final double kdt = 0.02;

    public class MotorIds {
        public static final int kShoulderId = 12; 
    }

    public class ShoulderConstants {
        public static final double kMaxVelocity = 50;
        public static final double kMaxAcceleration = 50;
        public static final double kSetpointTolerance = 0.1;

        public static final IdleMode kIdleMode = IdleMode.kBrake;

        public static final double kKp = 0.1;
        public static final double kKi = 0.0;
        public static final double kKd = 0.0;

        public static final int kCurrentLimit = 80;
        public static final boolean kInverted = false;
        public static final double kHomePosition = 0.0;
        public static final double kMaxPosition = 100.0;
        public static final double kMinPosition = 0.0;
        public static final double kPositionConversionFactor = 1;
    }
}
