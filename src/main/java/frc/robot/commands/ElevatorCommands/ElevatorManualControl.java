package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorManualControl extends Command {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier speedSupplier;

    public ElevatorManualControl(ElevatorSubsystem elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.manualControl(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.endManualControl(); // Stop the elevator when the command ends
    }
}