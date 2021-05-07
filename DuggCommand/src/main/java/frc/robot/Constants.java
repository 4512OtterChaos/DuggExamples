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
        public static final double kGearRatio = 8.46;
        public static final double kRampRaw = 0.08;
    }
    public static class LiftConstants{
        public static final int kMaxPos = 4300;
    }
}