package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;


public class armInCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();

    public armInCommand() {

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        arm.setArmMotor(Constants.armSpeed);
        SmartDashboard.putNumber("Arm Encoder", arm.getEncoderValue());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmMotor(0);
    }
}
