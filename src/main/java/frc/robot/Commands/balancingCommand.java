package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

import static frc.robot.Constants.Navmultiplier;
import static frc.robot.Constants.balanceTuner;


public class balancingCommand extends CommandBase {
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final NavX navX = NavX.getInstance();

    public balancingCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.navX);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double Pitch = NavX.getInstance().getPitch()-balanceTuner;

        SmartDashboard.putNumber("BalanceTuner", balanceTuner);
        if (Pitch > 0.5){
            driveTrain.setRightMotors(-Math.pow(Pitch, 0.35)*Navmultiplier);
            driveTrain.setLeftMotors(-Math.pow(Pitch, 0.35)*Navmultiplier);
        } else if (Pitch < - 0.5){
            driveTrain.setRightMotors(Math.pow(Math.abs(Pitch), 0.35)*Navmultiplier);
            driveTrain.setLeftMotors(Math.pow(Math.abs(Pitch), 0.35)*Navmultiplier);
        }
    }

    @Override
    public boolean isFinished() {
        //return NavX.getInstance().getPitch() < 0.3 || NavX.getInstance().getPitch() > -0.3;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setRightMotors(0);
        driveTrain.setLeftMotors(0);
    }
}
