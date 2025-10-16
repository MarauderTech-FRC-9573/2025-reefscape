package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetpointCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double setpoint;

    public ElevatorSetpointCommand(ElevatorSubsystem elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetPosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // No action needed; subsystem will continue to hold at targetPosition
        
    }
}