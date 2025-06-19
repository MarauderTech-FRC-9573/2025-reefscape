package frc.robot.commands.PivotCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotManualControl extends Command {
    private final PivotSubsystem pivot;
    private final double speedSupplier;

    public PivotManualControl(PivotSubsystem pivotSubsystem, double speedSupplier) {
        this.pivot = pivotSubsystem;
        this.speedSupplier = speedSupplier;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        pivot.manualControl(speedSupplier);
    }
    
    @Override
    public void end(boolean interrupted) { 
        pivot.endManualControl(); 
    }

    
}
