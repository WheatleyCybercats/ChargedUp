package frc.robot.Commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class inchForwardCommand extends CommandBase {
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    public inchForwardCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveTrain.setRightMotors(-0.12);
        driveTrain.setLeftMotors(-0.12);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setRightMotors(0);
        driveTrain.setLeftMotors(0);
    }
}
