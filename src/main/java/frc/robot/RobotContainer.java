// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.*;
import frc.robot.Commands.Presets.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LemonLight;
import frc.robot.subsystems.NavX;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // A simple auto routine that drives forward a specified distance, and then stops.


    private final Joystick DriverJoystick = new Joystick(0);
    private final Joystick OperatorJoystick = new Joystick(1);
    private final DriveTrain TrainDrive = new DriveTrain();
    private final NavX xNav = new NavX();
    private final seekingCommand SC = new seekingCommand(TrainDrive, new LemonLight());
    private final balancingCommand BC = new balancingCommand(TrainDrive, xNav);
    /* button-activated commands that were originally bound to the driver controller during testing
    public final armOutCommand AO = new armOutCommand();
    public final armInCommand AI = new armInCommand();
    public final elevatorOutCommand EO = new elevatorOutCommand();
    public final elevatorInCommand EI = new elevatorInCommand();
     */
    public final highPresetCommand HP = new highPresetCommand();
    public final midPresetCommand LP = new midPresetCommand();
    public final resetCommand IN = new resetCommand();
    public final unlockResetCommand UN = new unlockResetCommand();
    public final clawCloseCommand CC = new clawCloseCommand();
    public final clawOpenCommand CO = new clawOpenCommand();
    public final armToFloorCommand FL = new armToFloorCommand();
    public final substationCommand SB = new substationCommand();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        int A = 1;
        int B = 2;
        int X = 3;
        int Y = 4;
        int leftTrig = 5;
        int rightTrig = 6;

        /** DRIVER CONTROLLER **/
        JoystickButton resetCommand = new JoystickButton(DriverJoystick, A);
        resetCommand.whenPressed(IN);
        JoystickButton balanceButton = new JoystickButton(DriverJoystick, B); // 2 = B
        balanceButton.whileTrue(BC);
        JoystickButton armToFloor = new JoystickButton(DriverJoystick, X);
        armToFloor.whenPressed(FL);
        JoystickButton substation = new JoystickButton(DriverJoystick, Y);
        substation.whenPressed(SB);


        /* button-activated commands that were originally bound to the driver controller during testing
        JoystickButton armOut = new JoystickButton(OperatorJoystick, Y; // 4 = Y
        armOut.whileTrue(AO);
        JoystickButton armIn = new JoystickButton(OperatorJoystick, X); // 3 = X
        armIn.whileTrue(AI);
        JoystickButton elevatorOut = new JoystickButton(OperatorJoystick, rightTrig);
        elevatorOut.whileTrue(EO);
        JoystickButton elevatorIn = new JoystickButton(OperatorJoystick, leftTrig);
        elevatorIn.whileTrue(EI);
         */

        /** OPERATOR CONTROLLER **/
        JoystickButton highPreset = new JoystickButton(OperatorJoystick, Y); // Y = 4
        highPreset.whenPressed(HP);
        JoystickButton lowPreset = new JoystickButton(OperatorJoystick, X); // X = 3
        lowPreset.whenPressed(LP);
        JoystickButton reset = new JoystickButton(OperatorJoystick, A); // A = 1
        reset.whenPressed(IN);
        JoystickButton floor = new JoystickButton(OperatorJoystick, B);
        floor.whenPressed(FL);
        JoystickButton unlockReset = new JoystickButton(OperatorJoystick, 8); // rightMiddle = 2
        unlockReset.whenPressed(UN);
        JoystickButton armToSubstation = new JoystickButton(OperatorJoystick, 7); // rightMiddle = 2
        armToSubstation.whenPressed(SB);
        //JoystickButton seekButton = new JoystickButton(OperatorJoystick, B);
        //seekButton.whileTrue(SC);
        JoystickButton clawOpen = new JoystickButton(OperatorJoystick, leftTrig);//close
        clawOpen.whileTrue(CO);
        JoystickButton closeClaw = new JoystickButton(OperatorJoystick, rightTrig);//open
        closeClaw.whileTrue(CC);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /*
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                Constants.DriveConstants.ksVolts,
                                Constants.DriveConstants.kvVoltSecondsPerMeter,
                                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.DriveConstants.kDriveKinematics,
                        10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        // Pass config
                        config);

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        exampleTrajectory,
                        TrainDrive::getPose,
                        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                        new SimpleMotorFeedforward(
                                Constants.DriveConstants.ksVolts,
                                Constants.DriveConstants.kvVoltSecondsPerMeter,
                                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.DriveConstants.kDriveKinematics,
                        TrainDrive::getWheelSpeeds,
                        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        TrainDrive::tankDriveVolts,
                        TrainDrive);

        // Reset odometry to the starting pose of the trajectory.
        TrainDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> TrainDrive.tankDriveVolts(0, 0));
    }

         */
        return new SequentialCommandGroup(new highPresetCommand().withTimeout(3), new clawOpenCommand().withTimeout(1.5), new resetCommand().withTimeout(3));
    }
}

