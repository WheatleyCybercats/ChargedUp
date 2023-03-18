package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


public class elevatorOutCommand extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();

    public elevatorOutCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setElevatorMotors(-Constants.elevator.elevatorSpeed);
        SmartDashboard.putNumber("Elevator Encoder", elevator.getEncoderValue()[2]);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorMotors(0);
    }
}
