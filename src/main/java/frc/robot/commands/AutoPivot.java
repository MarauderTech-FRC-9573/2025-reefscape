package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class AutoPivot extends Command {
    private Pivot pivot;
    
    private boolean shouldFinish;

    public AutoPivot(Pivot pivot, boolean shouldFinish) {
        this.pivot = pivot;

        this.shouldFinish = shouldFinish;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        //pivot.run(PivotConstants.L2_POSITION);
        pivot.run(PivotConstants.L2_POSITION);
        //manipulator.runForward(ManipulatorConstants.CORAL_SCORE_SPEED);
    }

    @Override
    public void end(boolean isInterrupted) {
        pivot.stop();
        //manipulator.stop();
        //pivot.stop();
    }
}
