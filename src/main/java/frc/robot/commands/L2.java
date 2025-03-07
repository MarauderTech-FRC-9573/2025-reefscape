package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class L2 extends Command {
        Elevator elevator;

    public L2(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        elevator.run(ElevatorConstants.L2_ENCODER);
    }

    
}
