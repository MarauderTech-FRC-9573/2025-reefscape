package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSetpointCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final Double setpoint;

    public ElevatorSetpointCommand(ElevatorSubsystem elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.moveToSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.manualControl(0); // Stop the elevator when the command ends
    }
}