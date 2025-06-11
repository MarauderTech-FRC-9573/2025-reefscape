package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.ManipulatorConstants;;

public class Coral extends Command {
    Elevator elevator;
    int level;

    public Coral(Elevator elevator, int level) {
        this.elevator = elevator;
        this.level = level;
    }

    @Override
    public void initialize() {
        elevator.runHeight(level);
    }

    @Override
    public void execute() {
        //pivot.run(PivotConstants.CORAL_STATION_ENCODER);
        manipulator.runCoral(level);
    }
    
}