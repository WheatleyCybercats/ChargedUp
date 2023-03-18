package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;


public class autoHighPreset extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    public autoHighPreset() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.elevator);
    }

    @Override
    public void initialize() {
        //Constants.elevEncoderValue = 0;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("HP elevator encoder", Constants.elevEncoderValue);
        if(Constants.elevEncoderValue > Constants.preset.elevatorHighPreset + 3.5){
            elevator.setElevatorMotors(-Constants.elevator.elevatorSpeed);
        }

        SmartDashboard.putNumber("HP arm encoder", Constants.armEncoderValue);
        if (Constants.armEncoderValue > Constants.preset.armHighPreset + 2){
            arm.setArmMotor(-Constants.armSpeed);
        }

    }

    @Override
    public boolean isFinished() {
        double armEncoderValue = Constants.armEncoderValue;
        double elevEncoderValue = Constants.elevEncoderValue;

        if (elevEncoderValue <= -38) {
            return true;
        }
        if (armEncoderValue <= Constants.preset.armHighPreset + 4) {
            return true;
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
