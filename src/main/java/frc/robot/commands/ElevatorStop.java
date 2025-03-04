package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorStop extends Command {
    Elevator elevator;
    
    public ElevatorStop(Elevator elevator) {
        this.elevator = elevator;
    }    

    @Override
    public void execute() {
        elevator.stop();
    }
}
