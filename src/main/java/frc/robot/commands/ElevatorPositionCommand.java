package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorPositionCommand extends Command {
    private Elevator elevator;

    private double targetPos;

    private boolean shouldFinish;


    public ElevatorPositionCommand(Elevator elevator, double targetPos) {
        this.elevator = elevator;
        this.targetPos = targetPos;
        this.shouldFinish = shouldFinish;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        //pivot.run(PivotConstants.L2_POSITION);
        elevator.run(targetPos);
        //manipulator.runForward(ManipulatorConstants.CORAL_SCORE_SPEED);
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.stop();
        //manipulator.stop();
        //pivot.stop();
    }

    @Override
    public boolean isFinished() {
        return shouldFinish && elevator.atSetpoint();
    }
}
