package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class ElevConstants{
    public static final int kMaxEdges = 4300;
    public static final double kLEDBottomPos = 3.4;
    public static final double kLEDTopPos = 43.5;
    public static final int kEdgesPerRev = 360;
    public static final double kInchesTravelHeight = 82.5;

    public static final double kP = 1.75;
    public static final double kI = 0;
    public static final double kD = 0;
    
    public static final double kStaticFF = 0;
    public static final double kVelocityFF = 0.3;
    public static final double kAccelFF = 0;

    public static final Constraints kConstraintsInches = new Constraints(
        30, // max velocity
        50 // max acceleration
        );
}
