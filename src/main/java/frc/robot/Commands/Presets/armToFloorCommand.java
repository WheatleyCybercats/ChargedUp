package frc.robot.Commands.Presets;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;


public class armToFloorCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();

    public armToFloorCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("(floor) arm encoder", Constants.armEncoderValue);
        if (Constants.armEncoderValue > Constants.preset.armFloorPreset + 2){
            arm.setArmMotor(-Constants.armSpeed);
        }

        if (Constants.elevEncoderValue > Constants.preset.elevatorFloorPreset + 2){
            arm.setArmMotor(-Constants.armSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        double armEncoderValue = Constants.armEncoderValue;
        if(armEncoderValue <= Constants.preset.armFloorPreset + 1.5) {
            return Constants.elevEncoderValue <= Constants.preset.elevatorFloorPreset + 1.5;
        }
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmMotor(0);
    }
}
