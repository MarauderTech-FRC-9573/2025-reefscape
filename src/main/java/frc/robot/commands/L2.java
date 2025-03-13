package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

public class L2 extends Command {
        Elevator elevator;
        Pivot pivot;
        Manipulator manipulator;

    public L2(Elevator elevator, Manipulator manipulator, Pivot pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.manipulator = manipulator;
    }

    @Override
    public void execute() {
        elevator.run(ElevatorConstants.L2_ENCODER);
        pivot.run(PivotConstants.PIVOT_L3);
        manipulator.runForward(1);

    }

    
}
