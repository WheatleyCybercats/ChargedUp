package frc.robot.Commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LemonLight;

import static frc.robot.Constants.*;

public class autoCubeCommand extends CommandBase{
    private final DriveTrain driveTrain;
    private final LemonLight light;

    public autoCubeCommand(DriveTrain driveTrain, LemonLight light) {
        this.driveTrain = driveTrain;
        this.light = light;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double[] LemonValues = light.readValues();
        //light.smartDashboard(LemonValues);
        double[] lr = lr(LemonValues);
        double distance = calculateDistance();
        lr[0] += fb(distance);
        lr[1] += fb(distance);
        if(lr[0] > 1){
            lr[1] -= lr[0]-1;
            lr[0] = 1;
        } else if(lr[1] > 1){
            lr[0] -= lr[1]-1;
            lr[1] = 1;
        }
        driveTrain.setLeftMotors(-Math.sqrt(lr[1])*LLmultiplier);
        driveTrain.setRightMotors(-Math.sqrt(lr[0])*LLmultiplier);
    }

    @Override
    public boolean isFinished() {
        //return calculateDistance() == desiredDistance;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
           // driveTrain.setLeftMotors(0);
           // driveTrain.setRightMotors(0);
    }

    public static double calculateDistance() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        NetworkTableEntry tv = table.getEntry("tv");
        double target = tv.getDouble(0);
        if(target != 0) {

// how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = angleOfLL;

// distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = 5.25;

// distance from the target to the floor
            double goalHeightInches = heightOfTarget;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//calculate distance

            double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            SmartDashboard.putNumber("Distance", distance);
            return distance;
        }
        SmartDashboard.putNumber("Distance", 0);
        return 0;
    }
    public double[] lr(double[] LemonValues) {
        double[] lr = {0, 0};
        if (LemonValues[0] > 2.0) {
            lr[0] = (Math.sqrt(Math.abs(LemonValues[0] / 27)) * (LLmultiplier*2));
            SmartDashboard.putNumber("tx>0", Math.sqrt(Math.abs(LemonValues[0] / 27)) * (LLmultiplier*2));
        } else if (LemonValues[0] < -2.0) {
            lr[1] = (Math.sqrt(Math.abs(LemonValues[0] / 27)) * (LLmultiplier*2));
            SmartDashboard.putNumber("tx<0", Math.sqrt(Math.abs(LemonValues[0] / 27)) * (LLmultiplier*2));
        }
        return lr;
    }
    public double fb(double distance) {
        if (distance > desiredDistance && distance != 0){
            return (Math.sqrt(distance/100) * LLmultiplier);
        }
        return 0;
    }
}

