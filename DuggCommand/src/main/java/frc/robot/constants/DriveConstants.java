package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class DriveConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kTrackWidthMeters = Units.inchesToMeters(23);
    public static final double kGearRatio = 8.46;
    public static final double kRampRaw = 0.08;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    
    public static final double kStaticFF = 0;
    public static final double kVelocityFF = 0;
    public static final double kAccelFF = 0;

    public static final double kMaxAccelMeters = 0;
    public static final double kMaxVelMeters = 0;

    public static final double kMaxVoltage = 12;
}
