// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.*;
import frc.robot.Commands.Autos.*;
import frc.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    public Command autonomousCommand;

    private RobotContainer robotContainer;

    private final Joystick DriverJoystick = new Joystick(0);
    private final Joystick OperatorJoystick = new Joystick(1);
    private final frc.robot.subsystems.DriveTrain DriveTrain = new DriveTrain();
    private final LemonLight lemonlight = new LemonLight();
    private final NavX navX = new NavX();
    private final seekingCommand SC = new seekingCommand(DriveTrain, lemonlight);
    private final Arm arm = Arm.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    Thread m_visionThread;

    private static final String sideAuto1 = "sideAuto1";
    private static final String midAuto1 = "midAuto1";
    private static final String sideAuto2 = "sideAuto2";
    private String selectedAuto;
    private final SendableChooser<String> chooser = new SendableChooser<>();


    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        // instantiate the command used for the autonomous period

        robotContainer = new RobotContainer();

        /***********SENDABLE CHOOSER AUTOS***********/
        chooser.setDefaultOption("SIDE | 1GP+mobility", sideAuto1);
        chooser.addOption("SIDE | 2GP+mobility", sideAuto2);
        chooser.addOption("MID | 1GP+balance", midAuto1);
        SmartDashboard.putData("Auto Choices!", chooser);


/*
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

 */


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

        SmartDashboard.putNumber("Pitch", NavX.getInstance().getPitch());
        SmartDashboard.putNumber("Roll", NavX.getInstance().getRoll());
        /*
       // double armEncoderValue = Constants.armEncoderValue;
        double elevEncoderValue = Constants.elevEncoderValue;
        if (elevEncoderValue <= Constants.preset.elevatorHighPreset + 3) {
            elevator.setElevatorMotors(0);
        }
         */
        arm.armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //SmartDashboard.putNumber("Odometry X", DriveTrain.getPose().getX());
        //SmartDashboard.putNumber("Odometry Y", DriveTrain.getPose().getY());



        Constants.armEncoderValue = arm.getEncoderValue();
        Constants.elevEncoderValue = elevator.getEncoderValue()[0];

        SmartDashboard.putNumber("Left motor encoder", DriveTrain.getLeftEncoderValue());
        SmartDashboard.putNumber("Right motor encoder", DriveTrain.getRightEncoderValue());

    }


    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}


    @Override
    public void disabledPeriodic() {}


    /** This autonomous runs the +autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit()
    {
        DriveTrain.setLeftEncoderValue(0);
        DriveTrain.setRightEncoderValue(0);

        Constants.balanceTuner = navX.getRoll();

        Constants.armEncoderValue = 0;
        Constants.elevEncoderValue = 0;

        String selectedAuto = chooser.getSelected();
        SmartDashboard.putString("Selected Auto: ", selectedAuto);

        //clawOpenCommand CO = new clawOpenCommand();

        switch (selectedAuto) {
            case sideAuto1:
                    autonomousCommand = new SequentialCommandGroup(new autoHighPreset(), new clawOpenCommand().withTimeout(.5).
                        andThen(new autoReset()).andThen(new driveMetersCommand(4)));
                break;
            case sideAuto2:
                autonomousCommand = new SequentialCommandGroup(new autoHighPreset(), new clawOpenCommand().withTimeout(.5),
                        new autoReset(), new driveMetersCommand(6), new turnDegrees(180), new autoArmToFloor().withTimeout(1.3),
                        new clawOpenCommand().withTimeout(.5), new autoReset(), new driveMetersCommand(0.1), new clawCloseCommand().withTimeout(.5),
                        new turnDegrees(180), new driveMetersCommand(6), new autoHighPreset(), new clawOpenCommand().withTimeout(.5));
            case midAuto1:
            default:
                autonomousCommand = new SequentialCommandGroup(new autoHighPreset(), new clawOpenCommand().withTimeout(.5),
                        new autoReset(), new driveBackwardOverRamp(), new driveForwardOverRamp(), new balancingCommand());
                /******drive forward auto broken - negative amounts**/
                break;


                /** with .andThen()
                 *
                 * autonomousCommand = new SequentialCommandGroup(new autoHighPreset(), new clawOpenCommand().withTimeout(.5).
                 *                         andThen(new autoReset()).andThen(new driveBackwardMobility()).andThen(new turnDegrees()).andThen(new autoArmToFloor().withTimeout(1.3)).
                 *                         andThen(new clawOpenCommand().withTimeout(.5)).andThen(new autoInchForward()).andThen(new clawCloseCommand().withTimeout(.5)).
                 *                         andThen(new turnDegrees()).andThen(new driveForwardToComm()).andThen(new autoHighPreset(), new clawOpenCommand().withTimeout(.5)));
                 *
                 */
        }

        //autonomousCommand = new placeConeHighAuto();
        /**working auto before sendable chooser*/ // ----> autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }



    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        //Create a driveDistance command for the drivetrain. Then in robotcontainer, return a sequential command that drives a certain distance and do something. Also should be chained together

        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("MATCH TIME", Timer.getMatchTime());

        Constants.armEncoderValue = arm.getEncoderValue();
        Constants.elevEncoderValue = elevator.getEncoderValue()[0];

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

        Constants.armEncoderValue = 0;
        Constants.elevEncoderValue = 0;
    }


    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        //SeekingCommand.calculateDistance();
        //balance.execute();


        Constants.armEncoderValue = arm.getEncoderValue();
        Constants.elevEncoderValue = elevator.getEncoderValue()[0];

        double speed = DriverJoystick.getRawAxis(1) * Constants.dtMultiplier;
        if (speed < 0.12 && speed > -0.12)
            speed = 0;

        double turn = DriverJoystick.getRawAxis(4) * 0.43;
        if (turn > -0.12 && turn < 0.12)
            turn = 0;

        double left = speed - turn;
        double right = speed + turn;

        SmartDashboard.putNumber("RightMotorSpeed", right);
        SmartDashboard.putNumber("LeftMotorSpeed", left);

        DriveTrain.setLeftMotors(left);
        DriveTrain.setRightMotors(right);

        //Update elevator in robot periodic

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
