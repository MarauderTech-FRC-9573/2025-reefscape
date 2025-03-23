package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class PivotPositionCommand extends Command {
    private Pivot pivot;

    private double targetPos;
    
    private boolean shouldFinish;

    public PivotPositionCommand(Pivot pivot, double targetPos, boolean shouldFinish) {
        this.pivot = pivot;
        this.targetPos = targetPos;
        this.shouldFinish = shouldFinish;

        addRequirements(pivot);
    }

    public PivotPositionCommand(Pivot pivot, double targetPos) {
        this(pivot, targetPos, true);
    }

    @Override
    public void execute() {
        //pivot.run(PivotConstants.L2_POSITION);
        pivot.run(targetPos);
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
