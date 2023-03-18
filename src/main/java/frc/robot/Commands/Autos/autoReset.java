package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;


public class autoReset extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    public autoReset() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.elevator);
    }

    @Override
    public void initialize() {
        Constants.elevEncoderValue = elevator.getEncoderValue()[0];
        Constants.armEncoderValue = arm.getEncoderValue();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("reset elevator encoder", Constants.elevEncoderValue);
        if(Constants.elevEncoderValue < Constants.elevBottomConfig){
            elevator.setElevatorMotors(Constants.elevator.elevatorSpeed);
        }

        SmartDashboard.putNumber("reset arm encoder", Constants.armEncoderValue);
        if (Constants.armEncoderValue < Constants.armBottomConfig){
            arm.setArmMotor(Constants.armSpeed);
        }

        //drivetrain.arcadeDrive(-.15, 0);
        //drivetrain.arcadeDrive(-.15, 0);
    }

    @Override
    public boolean isFinished() {
        double armEncoderValue = Constants.armEncoderValue;
        double elevEncoderValue = Constants.elevEncoderValue;
        if(armEncoderValue >= -4) {
            return true;
        }
        if (elevEncoderValue >= -3) {
            return true;
        }
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmMotor(0);
        elevator.setElevatorMotors(0);
        //drivetrain.arcadeDrive(0,0);
        //drivetrain.arcadeDrive(0,0);
    }
}
