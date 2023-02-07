package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

import static frc.robot.Constants.Navmultiplier;
import static frc.robot.Constants.balanceTuner;


public class BalancingCommand extends CommandBase {
    private final DriveTrain driveTrain;
    private final NavX navX;

    public BalancingCommand(DriveTrain driveTrain, NavX navX) {
        this.driveTrain = driveTrain;
        this.navX = navX;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.navX);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double roll = NavX.getInstance().getRoll()-balanceTuner;
        SmartDashboard.putNumber("Pitch", NavX.getInstance().getPitch());
        SmartDashboard.putNumber("Roll", roll);
        SmartDashboard.putNumber("BalanceTuner", balanceTuner);
        if (roll > 0.5){
            driveTrain.setRightMotors(Math.pow(roll, 0.25)*Navmultiplier);
            driveTrain.setLeftMotors(Math.pow(roll, 0.25)*Navmultiplier);
        } else if (roll < - 0.5){
            driveTrain.setRightMotors(-Math.pow(Math.abs(roll), 0.25)*Navmultiplier);
            driveTrain.setLeftMotors(-Math.pow(Math.abs(roll), 0.25)*Navmultiplier);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
