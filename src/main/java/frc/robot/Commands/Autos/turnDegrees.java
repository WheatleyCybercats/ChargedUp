package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;


public class turnDegrees extends CommandBase {
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final NavX navX = new NavX();
    private double startingYaw;
    private final double projectedYaw;
    private final double desiredTurn;

    public turnDegrees(double desiredTurn) {
        this.desiredTurn = desiredTurn;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.navX);
        startingYaw = navX.getYaw();
        projectedYaw = (startingYaw + desiredTurn)%180;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveTrain.setRightMotors((Math.sqrt(Math.abs(navX.getYaw()-startingYaw)/180))*0.2);
        driveTrain.setRightMotors((-Math.sqrt(Math.abs(navX.getYaw()-startingYaw)/180))*0.2);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(navX.getYaw() - projectedYaw) < 0.3;
        // TODO: Make this return true when this Command no longer needs to run execute()
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setRightMotors(0);
    }
}
