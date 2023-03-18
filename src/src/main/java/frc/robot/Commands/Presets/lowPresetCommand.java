package frc.robot.Commands.Presets;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;


public class lowPresetCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    public lowPresetCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.elevator);
    }

    @Override
    public void initialize() {
        /*
        elevator.encoder1.setPosition(0);
        elevator.encoder2.setPosition(0);
        arm.armEncoder.setPosition(0);

         */
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("HP IN elevator encoder", Constants.elevEncoderValue);
        if(Constants.elevEncoderValue > Constants.lowPreset.elevatorLowPreset + 2){
            elevator.setElevatorMotors(-Constants.elevator.elevatorSpeed);
        }

        SmartDashboard.putNumber("HP IN arm encoder", Constants.armEncoderValue);
        if (Constants.armEncoderValue > Constants.lowPreset.armLowPreset + 2){
            arm.setArmMotor(-Constants.armSpeed);
        }

    }

    @Override
    public boolean isFinished() {
        double armEncoderValue = Constants.armEncoderValue;
        double elevEncoderValue = Constants.elevEncoderValue;
        if(armEncoderValue <= Constants.lowPreset.armLowPreset + 3) {
            if (elevEncoderValue <= Constants.lowPreset.elevatorLowPreset + 3) {
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
