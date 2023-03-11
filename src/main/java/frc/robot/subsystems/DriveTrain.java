// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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

    private final AHRS NavX = new AHRS();

    Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
    Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
    Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(5));


    DifferentialDrivePoseEstimator DDPE = new DifferentialDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, new NavX().ahrs.getRotation2d(), 0,0, new Pose2d(),localMeasurementStdDevs,visionMeasurementStdDevs);

    public DriveTrain() {

        left1.setNeutralMode(NeutralMode.Brake);
        left2.setNeutralMode(NeutralMode.Brake);
        left3.setNeutralMode(NeutralMode.Brake);
        right1.setNeutralMode(NeutralMode.Brake);
        right2.setNeutralMode(NeutralMode.Brake);
        right3.setNeutralMode(NeutralMode.Brake);

        leftMotors.setInverted(true);

        left1.setSelectedSensorPosition(0);
        right1.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        DDPE.addVisionMeasurement(new Pose2d(LemonLight.getInstance().getBotpose()[0], LemonLight.getInstance().getBotpose()[1], new Rotation2d(LemonLight.getInstance().getBotpose()[5])), Timer.getFPGATimestamp()-LemonLight.getInstance().getBotpose()[6]);
        DDPE.update(NavX.getRotation2d(), ticksToMeter()[1], ticksToMeter()[0]);
    }
    public void setRightMotors(double speed){
        rightMotors.set(speed);
    }

    public void setLeftMotors(double speed){
        leftMotors.set(speed);
    }

    public double getLeftMotorEncoderValue(){
        return left1.getSelectedSensorPosition();
    }

    public double getRightMotorEncoderValue(){
        return right1.getSelectedSensorPosition();
    }

    public Pose2d getLocation(){
        return DDPE.getEstimatedPosition();
    }
    public double[] ticksToMeter(){
        return new double[]{right1.getSelectedSensorPosition()/Constants.ticksPerMeter, left1.getSelectedSensorPosition()/Constants.ticksPerMeter};
    }

}