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
    public void initialize() {
        if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) > ElevatorConstants.L4_ENCODER) {
            while (Math.abs(elevator.leftMotor.getEncoder().getPosition()) > ElevatorConstants.L4_ENCODER) {
                elevator.runDown();
            }
        } else if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) < ElevatorConstants.L4_ENCODER) {
            while (Math.abs(elevator.leftMotor.getEncoder().getPosition()) < ElevatorConstants.L4_ENCODER) {
                elevator.runUp();
            }
        }else if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) == ElevatorConstants.L1_ENCODER) {
            elevator.stop();
        }
        elevator.stop();
    }

    @Override
    public void execute() {
        System.out.println("cowabunga");
    }

}
