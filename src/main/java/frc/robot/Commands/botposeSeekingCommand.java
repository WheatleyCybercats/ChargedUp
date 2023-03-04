package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LemonLight;


public class botposeSeekingCommand extends CommandBase {
    private final DriveTrain driveTrain = new DriveTrain();
    private final LemonLight lemonLight = LemonLight.getInstance();

    public botposeSeekingCommand(DriveTrain driveTrain, LemonLight lemonLight) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain, this.lemonLight);
    }

    @Override
    public void initialize() {

    }
    /*Figure out the closest targets to the robot based on the yaw. When presets are executed, draw an imaginary
      line based on the yaw and see if it is within a certain range to the target(high/mid, back/front correspondingly)

     */
    @Override
    public void execute() {
        double[] botpose = lemonLight.getBotpose();
        //x,y,z,roll,pitch,yaw
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
