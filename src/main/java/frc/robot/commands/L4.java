package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
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
    public void execute() {
        elevator.run(ElevatorConstants.L4_ENCODER);
        pivot.run(PivotConstants.PIVOT_L4);
        manipulator.runForward(1);
    }

}
