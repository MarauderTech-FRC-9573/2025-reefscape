package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class LowerAlgae extends Command {
    Elevator elevator;
    Manipulator manipulator;

    public LowerAlgae(Elevator elevator, Manipulator manipulator) {
        this.elevator = elevator;
        this.manipulator = manipulator;
    }

    @Override
    public void initialize() {
        // if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) < ElevatorConstants.L1_ENCODER) {
        //     while (Math.abs(elevator.leftMotor.getEncoder().getPosition()) < ElevatorConstants.L1_ENCODER) {
        //         elevator.runUp();
        //     }
        // } else if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) > ElevatorConstants.L1_ENCODER) {
        //     while (Math.abs(elevator.leftMotor.getEncoder().getPosition()) > ElevatorConstants.L1_ENCODER) {
        //         elevator.runDown();
        //     }
        // } else if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) == ElevatorConstants.L1_ENCODER) {
        //     elevator.stop();
        // }
        // elevator.stop();
    } 

    @Override
    public void execute() {
        elevator.run(ElevatorConstants.LOWER_ALGAE_ENCODER);
        //manipulator.runForward(ManipulatorConstants.ALGAE_INTAKE_SPEED);
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.stop();
        //manipulator.stop();
    }

}