package frc.robot.Commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LemonLight;


public class SeekingCommand extends CommandBase{
    private final DriveTrain driveTrain;

    public SeekingCommand(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        LemonLight LL = new LemonLight();
        double[] LemonValues = LL.readValues();
        double multiplier = 0.2;
        if (LemonValues[3] == 1.0){
            SmartDashboard.putString("Running", "True");
            while(LemonValues[0] > 0 || LemonValues[0] < 0) {
                if (LemonValues[0] > 0) {
                    driveTrain.setRightMotors(Math.sqrt(Math.abs(LemonValues[0]/27))*multiplier);
                    SmartDashboard.putNumber("tx>0", Math.sqrt(Math.abs(LemonValues[0]/27))*multiplier);
                }
                else {
                    driveTrain.setLeftMotors(Math.sqrt(Math.abs(LemonValues[0]/27))*multiplier);
                    SmartDashboard.putNumber("tx<0", Math.sqrt(Math.abs(LemonValues[0]/27))*multiplier);
                }
            }
            while(LemonValues[1] > 0 || LemonValues[1] < 0) {
                if (LemonValues[1] > 0) {
                    driveTrain.setRightMotors(Math.sqrt(Math.abs(LemonValues[1]/20.5))*multiplier);
                    driveTrain.setLeftMotors(Math.sqrt(Math.abs(LemonValues[1]/20.5))*multiplier);
                    SmartDashboard.putNumber("ty>0", Math.sqrt(Math.abs(LemonValues[1]/20.5))*multiplier);
                }else{
                    driveTrain.setRightMotors(-Math.sqrt(Math.abs(LemonValues[1]/20.5))*multiplier);
                    driveTrain.setLeftMotors(-Math.sqrt(Math.abs(LemonValues[1]/20.5))*multiplier);
                    SmartDashboard.putNumber("ty<0", -Math.sqrt(Math.abs(LemonValues[1]/20.5))*multiplier);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

    public void aim(double[] LemonValues){

    }
}
