package frc.robot.Commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;


public class unlockResetCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    public unlockResetCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmMotor(0);
        elevator.setElevatorMotors(0);
    }
}
