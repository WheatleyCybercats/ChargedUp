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
    public static final double armSpeed = 0.19;
    public static final class elevator{
        public static final double elevMultiplier = 0.1;
        public static final double elevatorSpeed = 0.38;
    }
    public static double armEncoderValue = arm.getEncoderValue();
    public static double elevEncoderValue = elev.getEncoderValue()[2];
    public static double armBottomConfig = -0.5;
    public static double elevBottomConfig = -0.1;
    public static final class highPreset{
        public static final double armHighPreset = -21.5; //encoder is -22 at optimal point
        public static final double elevatorHighPreset = -46.9; //encoder is -44 at optimal point
    }
    public static final class lowPreset{
        public static final double armLowPreset = -22.5;
        public static final double elevatorLowPreset = -25;
    }
    public static double LLmultiplier = 0.35;
    public static double Navmultiplier = 0.04;
    public static double desiredDistance = 50;
    public static int heightOfTarget = 44;
    public static double angleOfLL = 34.3;
    public static double balanceTuner = 0;

}
