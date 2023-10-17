package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;


public class driveBackwardOverRamp extends CommandBase {
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final NavX navX = new NavX();

    public driveBackwardOverRamp() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.navX);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //if(Timer.getMatchTime() >= 3){
        if (driveTrain.getRightEncoderValue() > 100_000){
            SmartDashboard.putNumber("(AUTO) Right DT Encoder", driveTrain.getRightEncoderValue());
            driveTrain.setRightMotors(0.15);
        }

        if(driveTrain.getLeftEncoderValue() > 100_000){
            SmartDashboard.putNumber("(AUTO) Left DT Encoder", driveTrain.getLeftEncoderValue());
            driveTrain.setLeftMotors(0.15);
        }
    }

    @Override
    public boolean isFinished() {
        if (driveTrain.getRightEncoderValue() < 100_000 || driveTrain.getLeftEncoderValue() < 100_000){
            return true;
        }
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setRightMotors(0);
        driveTrain.setLeftMotors(0);
    }
}
