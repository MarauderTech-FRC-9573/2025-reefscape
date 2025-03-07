package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class L3 extends Command {
        Elevator elevator;

    public L3(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        elevator.run(ElevatorConstants.L3_ENCODER);
    }

    
}
