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
    public void initialize() {
        if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) < ElevatorConstants.L3_ENCODER) {
            while (Math.abs(elevator.leftMotor.getEncoder().getPosition()) < ElevatorConstants.L3_ENCODER) {
                elevator.runUp();            
            }
            while (Math.abs(elevator.leftMotor.getEncoder().getPosition()) > ElevatorConstants.L3_ENCODER) {
                elevator.runDown();
            }
        } else if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) == ElevatorConstants.L3_ENCODER) {
            elevator.stop();
        }
        elevator.stop();
    }

    @Override
    public void execute() {
        System.out.println("cowabunga L3");
    }

}