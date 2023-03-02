// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.Presets.highPresetCommand;
import frc.robot.commands.Presets.lowPresetCommand;
import frc.robot.commands.Presets.resetCommand;
import frc.robot.commands.Presets.unlockResetCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LemonLight;
import frc.robot.subsystems.NavX;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    private final Joystick DriverJoystick = new Joystick(0);
    private final Joystick OperatorJoystick = new Joystick(1);
    private final DriveTrain TrainDrive = new DriveTrain();
    private final NavX xNav = new NavX();
    private final seekingCommand SC = new seekingCommand(TrainDrive, new LemonLight());
    private final balancingCommand BC = new balancingCommand(TrainDrive, xNav);
    /*
    public final armOutCommand AO = new armOutCommand();
    public final armInCommand AI = new armInCommand();
    public final elevatorOutCommand EO = new elevatorOutCommand();
    public final elevatorInCommand EI = new elevatorInCommand();

     */
    public final highPresetCommand HP = new highPresetCommand();
    public final lowPresetCommand LP = new lowPresetCommand();
    public final resetCommand IN = new resetCommand();
    public final unlockResetCommand UN = new unlockResetCommand();
    public final clawCloseCommand CC = new clawCloseCommand();
    public final clawOpenCommand CO = new clawOpenCommand();

    HashMap<String, Command> eventMap = new HashMap<>();
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings()
    {
        JoystickButton seekButton = new JoystickButton(DriverJoystick, 3); // 1 = A
        seekButton.whileTrue(SC);
        JoystickButton balanceButton = new JoystickButton(DriverJoystick, 2); // 2 = B
        balanceButton.whileTrue(BC);
        /*
        JoystickButton armOut = new JoystickButton(OperatorJoystick, 4); // 4 = Y
        armOut.whileTrue(AO);
        JoystickButton armIn = new JoystickButton(OperatorJoystick, 3); // 3 = X
        armIn.whileTrue(AI);
        JoystickButton elevatorOut = new JoystickButton(OperatorJoystick, 6); // 3 = X
        elevatorOut.whileTrue(EO);
        JoystickButton elevatorIn = new JoystickButton(OperatorJoystick, 5); // 3 = X
        elevatorIn.whileTrue(EI);
         */

        /** Operator **/
        JoystickButton highPreset = new JoystickButton(OperatorJoystick, 4); // Y = 4
        highPreset.whenPressed(HP);
        JoystickButton lowPreset = new JoystickButton(OperatorJoystick, 3); // X = 3
        lowPreset.whenPressed(LP);
        JoystickButton reset = new JoystickButton(OperatorJoystick, 1); // A = 1
        reset.whenPressed(IN);
        JoystickButton unlockReset = new JoystickButton(OperatorJoystick, 2); // B = 2
        unlockReset.whenPressed(UN);
        JoystickButton clawOpen = new JoystickButton(DriverJoystick, 6);//close
        clawOpen.whileTrue(CO);
        JoystickButton closeClaw = new JoystickButton(OperatorJoystick, 5);//open
        closeClaw.whileTrue(CC);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
        public Command getAutonomousCommand() {
            PathPlannerTrajectory trajectory = PathPlanner.loadPath("aton", new PathConstraints(4, 3));
            return TrainDrive.followTrajectoryCommand(trajectory, true);
        }
    }
