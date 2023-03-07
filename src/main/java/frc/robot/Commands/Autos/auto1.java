package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NavX;


public class auto1 extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final NavX navX = NavX.getInstance();
    //private final

    public auto1() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.driveTrain, this.elevator, this.navX);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute(){
            driveTrain.setRightMotors(0.3);
            driveTrain.setLeftMotors(0.3);
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() >= 3){
            return true;
        }
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmMotor(0);
        elevator.setElevatorMotors(0);
        driveTrain.setLeftMotors(0);
        driveTrain.setRightMotors(0);
    }
}
