// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public static final double kRobotDelta = 0.02;

    public static class DriveConstants{
        public static final double kWheelRadiusMeters = Units.inchesToMeters(6);
        public static final double kTrackWidthMeters = Units.inchesToMeters(23);
        public static final double kGearRatio = 8.46;
        public static final double kRampRaw = 0.08;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kMaxAccelMeters = 0;
        public static final double kMaxVelMeters = 0;
        public static final double kStaticFF = 0;
        public static final double kVelocityFF = 0;
        public static final double kAccelFF = 0;
    }
    public static class ElevConstants{
        public static final int kMaxEdges = 4300;
        public static final double kLEDBottomPos = 3.4;
        public static final double kLEDTopPos = 43.5;
        public static final int kEdgesPerRev = 360;
        public static final double kInchesTravelHeight = 82.5;
    }
}
