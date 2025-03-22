package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class AutoPivotL4 extends Command {
    private Pivot pivot;
    
    private boolean shouldFinish;

    public AutoPivotL4(Pivot pivot, boolean shouldFinish) {
        this.pivot = pivot;

        this.shouldFinish = shouldFinish;

        addRequirements(pivot);
    }

    @Override
    public void execute() {
        //pivot.run(PivotConstants.L2_POSITION);
        pivot.run(PivotConstants.L4_POSITION);
        //manipulator.runForward(ManipulatorConstants.CORAL_SCORE_SPEED);
    }

    @Override
    public void end(boolean isInterrupted) {
        pivot.stop();
        //manipulator.stop();
        //pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return shouldFinish && pivot.atSetpoint();
    }
}
