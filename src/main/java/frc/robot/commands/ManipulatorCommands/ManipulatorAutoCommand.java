package frc.robot.commands.ManipulatorCommands;

import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;


public class ManipulatorAutoCommand extends WaitCommand{
    private final Manipulator manipulator;
    public static double seconds = 0.5;

    public ManipulatorAutoCommand(Manipulator manipulator) {
        super(seconds);
        this.manipulator = manipulator;
    }

    @Override
    public void execute() {
        manipulator.runForward(ManipulatorConstants.CORAL_SCORE_SPEED);
    }
    
}
