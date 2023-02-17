package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class driveBoiCommand extends CommandBase {
    private final DriveTrain driveTrain;
    private double startTime;

    public driveBoiCommand(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if((Timer.getFPGATimestamp() - startTime) > 3){
            driveTrain.setRightMotors(0.3);
            driveTrain.setLeftMotors(0.3);
        }
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > 3;
        // TODO: Make this return true when this Command no longer needs to run execute()
    }

    @Override
    public void end(boolean interrupted) {

    }
}
