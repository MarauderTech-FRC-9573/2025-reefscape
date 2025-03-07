package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class L4 extends Command {
    Elevator elevator;

    public L4(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void execute() {
        elevator.run(ElevatorConstants.L4_ENCODER);
    }

}
