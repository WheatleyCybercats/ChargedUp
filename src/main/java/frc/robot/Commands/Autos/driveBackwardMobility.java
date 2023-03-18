package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;


public class driveBackwardMobility extends CommandBase {
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final NavX navX = new NavX();

    public driveBackwardMobility() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.navX);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveTrain.setRightMotors(0.4);
        driveTrain.setLeftMotors(0.4);
    }

    @Override
    public boolean isFinished() {
        if (driveTrain.getRightEncoderValue() > 100_000 || driveTrain.getLeftEncoderValue() > 100_000)
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setRightMotors(0);
        driveTrain.setLeftMotors(0);
    }
}
