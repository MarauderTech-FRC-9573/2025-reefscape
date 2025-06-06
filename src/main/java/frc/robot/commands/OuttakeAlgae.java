package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.ManipulatorConstants;;

public class OuttakeAlgae extends Command {
    Elevator elevator;
    Manipulator manipulator; 
    Pivot pivot;

    public OuttakeAlgae(Elevator elevator, Manipulator manipulator, Pivot pivot) {
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        elevator.run(ElevatorConstants.CORAL_STATION_ENCODER);
    }

    @Override
    public void execute() {
        //pivot.run(PivotConstants.CORAL_STATION_ENCODER);
        //manipulator.runBack(ManipulatorConstants.ALGAE_SCORE_SPEED);
    }
    
}
