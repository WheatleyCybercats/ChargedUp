package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Presets.highPresetCommand;
import frc.robot.Commands.Presets.resetCommand;
import frc.robot.Commands.clawOpenCommand;

public class placeConeCommandGroup extends SequentialCommandGroup {
    public placeConeCommandGroup() {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new highPresetCommand(),
                new clawOpenCommand(),
                new resetCommand()
        );
    }
}