package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;


public class setColorCommand extends CommandBase {
    private final LEDs leds = LEDs.getInstance();
    private final int[] colors;

    public setColorCommand(int[] colors) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.leds);
        this.colors = colors;
    }

    @Override
    public void initialize() {
        leds.setColor(colors);
    }

    @Override
    public void execute() {

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
