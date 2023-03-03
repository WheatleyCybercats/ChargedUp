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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.seekingCommand;
import frc.robot.subsystems.*;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.nio.file.Path;

import static frc.robot.Constants.arm;
import static frc.robot.Constants.dtMultiplier;


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
    Thread m_visionThread;


    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        robotContainer = new RobotContainer();

        m_visionThread =
                new Thread(
                        () -> {
                            // Get the UsbCamera from CameraServer
                            UsbCamera camera = CameraServer.startAutomaticCapture();
                            // Set the resolution
                            camera.setResolution(640, 480);

                            // Get a CvSink. This will capture Mats from the camera
                            CvSink cvSink = CameraServer.getVideo();
                            // Set up a CvSource. This will send images back to the Dashboard
                            CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);

                            // Mats are very memory expensive. Let's reuse this Mat.
                            Mat mat = new Mat();

                            // This cannot be 'true'. The program will never exit if it is. This
                            // lets the robot stop this thread when restarting robot code or
                            // deploying.
                            while (!Thread.interrupted()) {
                                // Tell the CvSink to grab a frame from the camera and put it
                                // in the source mat.  If there is an error notify the output.
                                if (cvSink.grabFrame(mat) == 0) {
                                    // Send the output the error.
                                    outputStream.notifyError(cvSink.getError());
                                    // skip the rest of the current iteration
                                    continue;
                                }
                                // Put a rectangle on the image
                                //Imgproc.rectangle(
                                        //mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                                // Give the output stream a new image to display
                                outputStream.putFrame(mat);
                            }
                        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();


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
        /*
       // double armEncoderValue = Constants.armEncoderValue;
        double elevEncoderValue = Constants.elevEncoderValue;
        if (elevEncoderValue <= Constants.preset.elevatorHighPreset + 3) {
            elevator.setElevatorMotors(0);
        }

         */
        arm.armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        SmartDashboard.putNumber("Odometry X", DriveTrain.getPose().getX());
        SmartDashboard.putNumber("Odometry Y", DriveTrain.getPose().getY());


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
/*
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
*/
        Constants.balanceTuner = navX.getRoll();
    }


    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
/*
        if(Timer.getFPGATimestamp() >= 3){
            DriveTrain.setRightMotors(-0.3);
            DriveTrain.setLeftMotors(-0.3);
        }

 */
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

        if (Constants.elevEncoderValue < -10){
            Constants.dtMultiplier = 0.25;
        }
        else {
            Constants.dtMultiplier = 1;
        }

        SmartDashboard.putNumber("RightMotorSpeed", right);
        SmartDashboard.putNumber("LeftMotorSpeed", left);

        DriveTrain.setLeftMotors(left*dtMultiplier);
        DriveTrain.setRightMotors(right*dtMultiplier);

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
