package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PivotSetpointCommand extends Command {
    private final PivotSubsystem pivot;
    private final double setpoint;

    public PivotSetpointCommand(PivotSubsystem pivot, double setpoint) {
        this.pivot = pivot;
        this.setpoint = setpoint;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setTargetPosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return pivot.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // No action needed; subsystem will continue to hold at targetPosition
    }
}