package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

public class L4 extends Command {
    Elevator elevator;
    Manipulator manipulator;
    Pivot pivot;

    public L4(Elevator elevator, Manipulator manipulator, Pivot pivot) {
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.pivot = pivot;
    }


    @Override
    public void initialize() {
        pivot.run(PivotConstants.L4_POSITION);
        // if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) > ElevatorConstants.L4_ENCODER) {
        //     while (Math.abs(elevator.leftMotor.getEncoder().getPosition()) > ElevatorConstants.L4_ENCODER) {
        //         elevator.runDown();
        //     }
        // } else if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) < ElevatorConstants.L4_ENCODER) {
        //     while (Math.abs(elevator.leftMotor.getEncoder().getPosition()) < ElevatorConstants.L4_ENCODER) {
        //         elevator.runUp();
        //     }
        // }else if (Math.abs(elevator.leftMotor.getEncoder().getPosition()) == ElevatorConstants.L1_ENCODER) {
        //     elevator.stop();
        // }
        // elevator.stop();
    }

    @Override
    public void execute() {
       elevator.run(ElevatorConstants.L4_ENCODER);
        //manipulator.runForward(ManipulatorConstants.CORAL_SCORE_SPEED);
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.stop();
        //manipulator.stop();
        pivot.stop();
    }
}
