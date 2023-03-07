// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.SeekingCommand;
import frc.robot.subsystems.*;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.annotation.Target;
import java.nio.file.Path;
import java.util.TreeMap;

import static frc.robot.Constants.*;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private final SendableChooser<String> chooser = new SendableChooser<>();
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private final Joystick DriverJoystick = new Joystick(0);
    private final Joystick OperatorJoystick = new Joystick(1);
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private LemonLight lemonlight = new LemonLight();
    private NavX navX = new NavX();
    private SeekingCommand SC = new SeekingCommand(driveTrain, lemonlight);
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();

    public static Targets[] targets = new Targets[18];
    String selectedAuto;
    private double translation;


    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        if(Constants.color.equalsIgnoreCase("RED")){
            translation = 15.85;
        }

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        robotContainer = new RobotContainer();

        targets[0] = new Targets("1", 0.35 + translation, 0.50, 1.11);
        targets[1] = new Targets("2", 0.7 + translation, 0.5, 0.81);
        targets[2] = new Targets("3", 0.35 + translation, 1.05, 1.11);
        targets[3] = new Targets("4", 0.7 + translation, 1.05, 0.81);
        targets[4] = new Targets("5", 0.35 + translation, 1.61, 1.11);
        targets[5] = new Targets("6", 0.7 + translation, 1.61, 0.81);
        targets[6] = new Targets("7", 0.35 + translation, 2.2, 1.11);
        targets[7] = new Targets("8", 0.7 + translation, 2.2, 0.81);
        targets[8] = new Targets("9", 0.35 + translation, 2.75, 1.11);
        targets[9] = new Targets("10", 0.7 + translation, 2.75, 0.81);
        targets[10] = new Targets("11", 0.35 + translation, 3.3, 1.11);
        targets[11] = new Targets("12", 0.7 + translation, 3.3, 0.81);
        targets[12] = new Targets("13", 0.35 + translation, 3.85, 1.11);
        targets[13] = new Targets("14", 0.7 + translation, 3.85, 0.81);
        targets[14] = new Targets("15", 0.35 + translation, 4.4, 1.11);
        targets[15] = new Targets("16", 0.7 + translation, 4.4, 0.81);
        targets[16] = new Targets("17", 0.35 + translation, 5, 1.11);
        targets[17] = new Targets("18", 0.7 + translation, 5, 0.81);


        Constants.balanceTuner = navX.getRoll();
    }

    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
       // double armEncoderValue = Constants.armEncoderValue;
        double elevEncoderValue = Constants.elevEncoderValue;
        if (elevEncoderValue <= Constants.preset.elevatorHighPreset + 3) {
            elevator.setElevatorMotors(0);
        }

        arm.armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        LemonLight LL = new LemonLight();
        SmartDashboard.putNumber("Limelight Pose X", LL.getBotpose()[0]);
        SmartDashboard.putNumber("Limelight Pose Y", LL.getBotpose()[1]);
        SmartDashboard.putNumber("Limelight Pose Z", LL.getBotpose()[2]);

        chooser.addOption("Aton", "Aton");

        navXYaw = NavX.getInstance().getYaw();
        if(LL.getBotpose()[0] != 0){
            localLocation[0] = LL.getBotpose()[0];
        }
        if(LL.getBotpose()[1] != 0){
            localLocation[1] = LL.getBotpose()[1];
        }

        SmartDashboard.putNumber("Local X", localLocation[0]);
        SmartDashboard.putNumber("Local Y", localLocation[1]);
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
        selectedAuto = chooser.getSelected();

        autonomousCommand = robotContainer.getAutonomousCommand(selectedAuto);

        // schedule the autonomous command (example)
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }


    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

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

        if(speed > 0 || turn > 0)
            driveTrain.updateXY();
        double left = speed - turn;
        double right = speed + turn;

        SmartDashboard.putNumber("RightMotorSpeed", right);
        SmartDashboard.putNumber("LeftMotorSpeed", left);

        driveTrain.setLeftMotors(left*dtMultiplier);
        driveTrain.setRightMotors(right*dtMultiplier);

        Constants.armEncoderValue = arm.getEncoderValue();
        Constants.elevEncoderValue = elevator.getEncoderValue()[2];

        double elevatorMovement = OperatorJoystick.getRawAxis(5);
        if(elevatorMovement > -0.05 && elevatorMovement < 0.05)
            elevatorMovement = 0;

        elevator.setElevatorMotors(elevatorMovement*0.5);

        double armMovement = OperatorJoystick.getRawAxis(1);
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
