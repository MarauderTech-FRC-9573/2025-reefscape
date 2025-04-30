package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;;

public class AutoIntakeAlgae extends Command {
    Elevator elevator;
    Manipulator manipulator; 
    Pivot pivot;

    public AutoIntakeAlgae(Manipulator manipulator) {
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        manipulator.runForward(0.5);
        //manipulator.runBack(ManipulatorConstants.ALGAE_SCORE_SPEED);
    }
    
}