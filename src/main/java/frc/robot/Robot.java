// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.seekingCommand;
import frc.robot.subsystems.*;

import java.nio.file.Path;

import static frc.robot.Constants.arm;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private final Joystick DriverJoystick = new Joystick(0);
    private final Joystick OperatorJoystick = new Joystick(1);
    private final frc.robot.subsystems.DriveTrain DriveTrain = new DriveTrain();
    private LemonLight lemonlight = new LemonLight();
    private NavX navX = new NavX();
    private seekingCommand SC = new seekingCommand(DriveTrain, lemonlight);
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private PathPlannerTrajectory[] PT = new PathPlannerTrajectory[20];
    private int pathIndex = 0;
    private PathPlannerTrajectory aton = PathPlanner.loadPath("aton", new PathConstraints(3, 2));



    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        PT[0] = aton;
    }


    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }


    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}


    @Override
    public void disabledPeriodic() {}


    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit()
    {

        /*autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }

        Constants.balanceTuner = navX.getRoll();

         */


    }


    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        DriveTrain.setLeftMotors(0.31);
        DriveTrain.setRightMotors(0.3);

    }

    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        Constants.balanceTuner = navX.getRoll();

    }


    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        //SeekingCommand.calculateDistance();
        //balance.execute();

        double speed = DriverJoystick.getRawAxis(1) * 0.75;
        if (speed < 0.12 && speed > -0.12)
            speed = 0;

        double turn = DriverJoystick.getRawAxis(4) * 0.45;
        if (turn > -0.12 && turn < 0.12)
            turn = 0;

        double left = speed - turn;
        double right = speed + turn;

        SmartDashboard.putNumber("RightMotorSpeed", right);
        SmartDashboard.putNumber("LeftMotorSpeed", left);
        DriveTrain.setLeftMotors(left);
        DriveTrain.setRightMotors(right);

        Constants.armEncoderValue = arm.getEncoderValue();
        Constants.elevEncoderValue = elevator.getEncoderValue()[2];

        double elevatorMovement = OperatorJoystick.getRawAxis(1);
        if(elevatorMovement > -0.05 && elevatorMovement < 0.05)
            elevatorMovement = 0;

        elevator.setElevatorMotors(elevatorMovement*0.5);

        double armMovement = OperatorJoystick.getRawAxis(5);
        if(armMovement > -0.03 && armMovement < 0.03)
            armMovement = 0;

        arm.setArmMotor(armMovement*0.3);

        SmartDashboard.putNumber("Constants Arm", Constants.armEncoderValue);
        SmartDashboard.putNumber("Constants Elev", Constants.elevEncoderValue);
    }


    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }


    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
