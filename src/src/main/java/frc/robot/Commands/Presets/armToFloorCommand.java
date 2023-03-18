package frc.robot.Commands.Presets;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;


public class armToFloorCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    public armToFloorCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("FLOOR elevator encoder", Constants.elevEncoderValue);
        if(Constants.elevEncoderValue > Constants.preset.elevatorFloorPreset + 3.5){
            elevator.setElevatorMotors(-Constants.elevator.elevatorSpeed);
        }

        SmartDashboard.putNumber("floor arm encoder", Constants.armEncoderValue);
        if (Constants.armEncoderValue > Constants.preset.armFloorPreset + 2){
            arm.setArmMotor(-Constants.armSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        double armEncoderValue = Constants.armEncoderValue;
        double elevEncoderValue = Constants.elevEncoderValue;
        if(armEncoderValue <= Constants.preset.armFloorPreset + 3){
            if (elevEncoderValue <= Constants.preset.elevatorFloorPreset + 3.5) {
                return true;
            }
        }
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        arm.setArmMotor(0);
        elevator.setElevatorMotors(0);
    }
}
