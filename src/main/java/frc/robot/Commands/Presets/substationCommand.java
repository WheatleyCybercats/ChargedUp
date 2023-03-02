package frc.robot.Commands.Presets;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;


public class substationCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Claw claw = Claw.getInstance();

    public substationCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        claw.openClaw();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("(substation) arm encoder", Constants.armEncoderValue);
        if (Constants.armEncoderValue > Constants.preset.substationPreset + 2){
            arm.setArmMotor(-Constants.armSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        double armEncoderValue = Constants.armEncoderValue;
        if(armEncoderValue <= Constants.preset.substationPreset + 3) {
            return true;
        }
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
