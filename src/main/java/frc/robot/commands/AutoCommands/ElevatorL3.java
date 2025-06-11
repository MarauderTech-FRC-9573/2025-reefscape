package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorL3 extends Command {
    private ElevatorSubsystem elevator;



    public ElevatorL3(ElevatorSubsystem elevator) {
        this.elevator = elevator;


        addRequirements(elevator);
    }

    @Override
    public void execute() {
        //pivot.run(PivotConstants.L2_POSITION);
        elevator.run(ElevatorConstants.L3_ENCODER);
        //manipulator.runForward(ManipulatorConstants.CORAL_SCORE_SPEED);
    }

    @Override
    public void end(boolean isInterrupted) {
        elevator.stop();
        //manipulator.stop();
        //pivot.stop();
    }
}