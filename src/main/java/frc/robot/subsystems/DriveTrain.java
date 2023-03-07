// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
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

    // The gyro sensor
    private final AHRS NavX = new AHRS();

    // Odometry class for tracking robot pos


    /** Creates a new DriveSubsystem. */
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

    public double[] ticksToMeter(){
        return new double[]{right1.getSelectedSensorPosition()/Constants.ticksPerMeter, left1.getSelectedSensorPosition()/Constants.ticksPerMeter};
    }

    public void updateXY(){
        double deltaX = (Math.abs(ticksToMeter()[0]) + Math.abs(ticksToMeter()[1]))/2 * Math.cos(Constants.navXYaw);
        double deltaY = (Math.abs(ticksToMeter()[0]) + Math.abs(ticksToMeter()[1]))/2 * Math.sin(Constants.navXYaw);
        Constants.localLocation[0] += deltaX;
        Constants.localLocation[1] += deltaY;
    }
}