// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase{

    private final static DriveTrain INSTANCE = new DriveTrain();
    public static DriveTrain getInstance() {
        return INSTANCE;
    }

    WPI_TalonFX left1 = new WPI_TalonFX(1);
    WPI_TalonFX left2 = new WPI_TalonFX(2);
    WPI_TalonFX left3 = new WPI_TalonFX(3);
//----------------------------------------------------------------
    WPI_TalonFX right1 = new WPI_TalonFX(4);
    WPI_TalonFX right2 = new WPI_TalonFX(5);
    WPI_TalonFX right3 = new WPI_TalonFX(6);



    private final MotorControllerGroup leftMotors =
            new MotorControllerGroup(
                    left1,
                    left2,
                    left3);

    // The motors on the right side of the drive.
    private final MotorControllerGroup rightMotors =
            new MotorControllerGroup(
                    right1,
                    right2,
                    right3);

    // The robot's drive
    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    // The gyro sensor
    private final AHRS NavX = new AHRS();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;


    /** Creates a new DriveSubsystem. */
    public DriveTrain() {

        left1.setNeutralMode(NeutralMode.Brake);
        left2.setNeutralMode(NeutralMode.Brake);
        left3.setNeutralMode(NeutralMode.Brake);
        right1.setNeutralMode(NeutralMode.Brake);
        right2.setNeutralMode(NeutralMode.Brake);
        right3.setNeutralMode(NeutralMode.Brake);

        leftMotors.setInverted(true);
        //drive.setDeadband(0.08);
        resetEncoders();
        odometry =
                new DifferentialDriveOdometry(
                        NavX.getRotation2d(), left1.getSensorCollection().getIntegratedSensorPosition(), right1.getSensorCollection().getIntegratedSensorPosition());
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
                NavX.getRotation2d(), left1.getSensorCollection().getIntegratedSensorPosition(), right1.getSensorCollection().getIntegratedSensorPosition());
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
        return new DifferentialDriveWheelSpeeds(left1.getSelectedSensorVelocity(), right1.getSelectedSensorVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
                NavX.getRotation2d(), left1.getSensorCollection().getIntegratedSensorPosition(), right1.getSensorCollection().getIntegratedSensorPosition(), pose);
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
        left1.setSelectedSensorPosition(0);
        right1.setSelectedSensorPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (left1.getSensorCollection().getIntegratedSensorPosition() + right1.getSensorCollection().getIntegratedSensorPosition()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Double getLeftEncoder() {
        return left1.getSensorCollection().getIntegratedSensorPosition();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Double getRightEncoder() {
        return right1.getSensorCollection().getIntegratedSensorPosition();
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
        NavX.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return NavX.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -NavX.getRate();
    }

    public void setRightMotors(double speed){
        rightMotors.set(speed);
    }

    public void setLeftMotors(double speed){
        leftMotors.set(speed);
    }
}