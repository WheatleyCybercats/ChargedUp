// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase{
    private final WPI_TalonFX leftLeader = new WPI_TalonFX(1);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(2);
    private final WPI_TalonFX leftFollower2 = new WPI_TalonFX(3);
    //------------------------------------------------------------------------
    private final WPI_TalonFX rightLeader = new WPI_TalonFX(4);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(5);
    private final WPI_TalonFX rightFollower2 = new WPI_TalonFX(6);
    private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader,leftFollower,leftFollower2);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader,rightFollower,rightFollower2);


    public void setLeftMotors(double speed){leftGroup.set(speed);}
    public void setRightMotors(double speed){rightGroup.set(speed);}
    public double getLeftEncoderValue(){
        return leftLeader.getSelectedSensorPosition();
    }
    public double getRightEncoderValue(){
        return rightLeader.getSelectedSensorPosition();
    }
    public DriveTrain(){
        leftFollower.set(ControlMode.Follower, leftLeader.getDeviceID());
        rightFollower.set(ControlMode.Follower, rightLeader.getDeviceID());
        leftFollower2.set(ControlMode.Follower, leftLeader.getDeviceID());
        rightFollower2.set(ControlMode.Follower, rightLeader.getDeviceID());

        leftLeader.setInverted(true);
        leftFollower.setInverted(true);
        leftFollower2.setInverted(true);
        //-----------------------------------
        rightLeader.setInverted(false);
        rightFollower.setInverted(false);
        rightFollower2.setInverted(false);
        //-----------------------------------
        leftLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        leftFollower2.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
        rightFollower2.setNeutralMode(NeutralMode.Brake);

        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }
}