package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;


public class ManipulatorDirectionCommand extends Command {
    private Manipulator manipulator;
    private Pivot pivot;
    private double targetDir;

    public ManipulatorDirectionCommand(Manipulator manipulator, Pivot pivot, double targetDir) {
        this.manipulator = manipulator;
        this.pivot = pivot;
        this.targetDir = targetDir;

        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        //pivot.run(PivotConstants.L2_POSITION);
        manipulator.runForward(targetDir);
        //manipulator.runForward(ManipulatorConstants.CORAL_SCORE_SPEED);

        if (!pivot.getBeamBreak().isPressed()){
            while (!pivot.getBeamBreak().isPressed()) {
                manipulator.runBack(ManipulatorConstants.CORAL_INTAKE_SPEED);
            }
        } else {
            System.out.println("BEAM BREAK HIHIHI");
            manipulator.stop();
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        //elevator.stop();
        manipulator.stop();
        //pivot.stop();
    }
}
