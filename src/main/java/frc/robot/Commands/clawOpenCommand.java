package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;


public class clawOpenCommand extends CommandBase {
    private final Claw claw = Claw.getInstance();

    public clawOpenCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.claw);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        claw.openClaw();
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
