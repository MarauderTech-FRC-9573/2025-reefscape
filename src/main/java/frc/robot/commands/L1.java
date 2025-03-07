package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class L1 extends Command {
        Elevator elevator;

    public L1(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        elevator.run(ElevatorConstants.L1_ENCODER);
    }

    
}
