package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    private final WPI_TalonFX left1 = new WPI_TalonFX(1);
    private final WPI_TalonFX left2 = new WPI_TalonFX(2);
    private final WPI_TalonFX left3 = new WPI_TalonFX(3);
    //---------------------------------------------------------------
    private final WPI_TalonFX right1 = new WPI_TalonFX(4);
    private final WPI_TalonFX right2 = new WPI_TalonFX(5);
    private final WPI_TalonFX right3 = new WPI_TalonFX(6);

    private final static DriveTrain INSTANCE = new DriveTrain();
    public static DriveTrain getInstance() {
        return INSTANCE;
    }

    // The motors on the left side of the drive.
    private final MotorControllerGroup leftMotors =
            new MotorControllerGroup(left1, left2, left3);

    // The motors on the right side of the drive.
    private final MotorControllerGroup rightMotors =
            new MotorControllerGroup(
                    right1, right2, right3);

    private final Encoder leftEncoder = new Encoder(left1.getDeviceID(), left2.getDeviceID(), right3.getDeviceID());
    private final Encoder rightEncoder = new Encoder(right1.getDeviceID(), right2.getDeviceID(), right3.getDeviceID());

    // The robot's drive
    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    // The gyro sensor
    private final Gyro gyro = new AHRS(SPI.Port.kMXP);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotors.setInverted(true);

        // Sets the distance per pulse for the encoders
        leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);


        resetEncoders();
        odometry =
                new DifferentialDriveOdometry(
                        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
                gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    public void setLeftMotors(double speed){leftMotors.set(speed);}
    public void setRightMotors(double speed){rightMotors.set(speed);}

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if(isFirstPath){
                        this.resetOdometry(traj.getInitialPose());
                    }
                }),
                new PPRamseteCommand(
                        traj,
                        this::getPose, // Pose supplier
                        new RamseteController(),
                        new SimpleMotorFeedforward(Constants.Ks, Constants.Kv, Constants.Ka),
                        Constants.kDriveKinematics, // DifferentialDriveKinematics
                        this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                        new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                        this::tankDriveVolts, // Voltage biconsumer
                        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                        this // Requires this drive subsystem
                )
        );
    }

}