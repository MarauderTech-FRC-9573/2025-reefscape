package frc.robot.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class ManipulatorCommand extends Command {
    private final Manipulator manipulator;
    private final Double speed;

    public ManipulatorCommand(Manipulator manipulator, Double speed) {
        this.manipulator = manipulator;
        this.speed = speed;
        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        manipulator.manualControl(speed);
    }

    @Override
    public void end(boolean interrupted) {
        manipulator.manualControl(0); // Stop the manipulator when the command ends
    }
}