// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static Arm arm = Arm.getInstance();
    public static Elevator elev = Elevator.getInstance();

    //make a class for Arm
    public static final int armMotor = 5;
    public static final double armSpeed = 0.7;

    public static double dtMultiplier = 1;


    public static final class elevator{
        public static double elevMultiplier = 1;
        public static final double elevatorSpeed = 0.8;
    }
    public static double armEncoderValue = arm.getEncoderValue();
    public static double elevEncoderValue = elev.getEncoderValue()[2];
    public static double armBottomConfig = -5;
    public static double elevBottomConfig = -1;
    public static final class preset{
        public static final double armHighPreset = -31; // relative encoder is -35 at optimal point
        public static final double elevatorHighPreset = -37; // relative encoder is -45 at optimal point
        public static final double armMidPreset = -32;
        public static final double elevatorFloorPreset = -7;
        public static final double armFloorPreset = -50;
        public static final double substationPreset = -32;
    }
    public static double LLmultiplier = 0.15;
    public static double Navmultiplier = 0.04;
    public static double desiredDistance = 80;
    public static int heightOfTarget = 44;
    public static double angleOfLL = 34.3;
    public static double balanceTuner = 0;
    public static final class AutoConstants {
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.8;
        public static final double kMaxSpeedMetersPerSecond = 0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0;
    }
    public static final class DriveConstants{
        public static final double kPDriveVel = 0;
        public static final double kTrackwidthMeters = 0.625;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double ksVolts = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;
    }

    public static final class botpose{
        public static final class Target1{
            double x = 0;
            double y = 0;
            double z = 0;
        }
    }

}
