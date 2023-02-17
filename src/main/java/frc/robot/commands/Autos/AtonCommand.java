package frc.robot.commands.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Presets.highPresetCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LemonLight;
import frc.robot.subsystems.NavX;

import java.util.HashMap;


public class AtonCommand extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final Claw claw = Claw.getInstance();
    private final DriveTrain driveTrain = DriveTrain.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final LemonLight lemonLight = LemonLight.getInstance();
    private final NavX navX = NavX.getInstance();

    PathPlannerTrajectory aton;
    private int index = 0;

    public AtonCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm, this.claw, this.driveTrain, this.elevator, this.lemonLight, this.navX);
    }

    @Override
    public void initialize() {
        aton = PathPlanner.loadPath("aton", new PathConstraints(4, 3));
    }

    @Override
    public void execute() {
        PathPlannerTrajectory.PathPlannerState state = aton.getState(index);
        Pose2d position = state.poseMeters;
        if(!driveTrain.getPose().equals(position)){
            if(position.getX() > driveTrain.getPose().getX()) {
                driveTrain.arcadeDrive(Math.sqrt((Math.abs(position.getX()-driveTrain.getPose().getX()))), 0);
            }
            if(position.getY() > driveTrain.getPose().getY()) {
                driveTrain.arcadeDrive(0, (Math.abs(position.getY()-driveTrain.getPose().getY())));
            }
        }
        if(driveTrain.getPose().equals(position))
            index++;
    }

    @Override
    public boolean isFinished() {
        if(aton.getState(index) == aton.getEndState())
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
