package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class driveMetersCommand extends CommandBase {
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final double DriveAmount;
    private final double startingMeter;
    private final double projectedAmount;

    public driveMetersCommand(double driveAmount1) {
        DriveAmount = driveAmount1;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
        startingMeter = (Math.abs(driveTrain.ticksToMeter()[0]) + Math.abs(driveTrain.ticksToMeter()[1]))/2;
        projectedAmount = startingMeter + driveAmount1;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(DriveAmount != 0){
            if(DriveAmount > 0){
                driveTrain.setLeftMotors((Math.sqrt((projectedAmount-(Math.abs(driveTrain.ticksToMeter()[0]) + Math.abs(driveTrain.ticksToMeter()[1]))/2))/DriveAmount)*0.5);
                driveTrain.setRightMotors((Math.sqrt((projectedAmount-(Math.abs(driveTrain.ticksToMeter()[0]) + Math.abs(driveTrain.ticksToMeter()[1]))/2))/DriveAmount)*0.5);
            }
            if(DriveAmount < 0){
                driveTrain.setRightMotors((-Math.sqrt(Math.abs(((projectedAmount-(Math.abs(driveTrain.ticksToMeter()[0]) + Math.abs(driveTrain.ticksToMeter()[1]))/2))/DriveAmount)))*0.5);
                driveTrain.setLeftMotors((-Math.sqrt(Math.abs(((projectedAmount-(Math.abs(driveTrain.ticksToMeter()[0]) + Math.abs(driveTrain.ticksToMeter()[1]))/2))/DriveAmount)))*0.5);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(projectedAmount - (Math.abs(driveTrain.ticksToMeter()[0]) + Math.abs(driveTrain.ticksToMeter()[1])) / 2) < 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setLeftMotors(0);
        driveTrain.setRightMotors(0);
    }
}
