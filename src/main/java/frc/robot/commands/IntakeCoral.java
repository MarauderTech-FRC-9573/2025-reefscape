package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.ManipulatorConstants;;

public class IntakeCoral extends Command {
    Elevator elevator;
    Manipulator manipulator; 
    Pivot pivot;

    public IntakeCoral(Elevator elevator, Manipulator manipulator, Pivot pivot) {
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
        pivot.run(-6.0);
    }

    @Override
    public void execute() {
        //System.out.println("hi");
        elevator.run(0.0);
        if (!pivot.getBeamBreak().isPressed()){
            while (!pivot.getBeamBreak().isPressed()) {
                System.out.println("BEAM BREAK HIHIHI");
                manipulator.runBack(ManipulatorConstants.CORAL_BACKWARD_SPEED);
            }
        } else {
            manipulator.stop();
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.stop();
        manipulator.stop();
        pivot.stop();
    }
    
}
