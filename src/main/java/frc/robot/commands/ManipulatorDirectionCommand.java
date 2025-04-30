package frc.robot.commands;

import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;


public class ManipulatorDirectionCommand extends Command {
    private Manipulator manipulator;
    private SparkLimitSwitch beamBreaker;
    private double targetDir;

    public ManipulatorDirectionCommand(Manipulator manipulator, SparkLimitSwitch beamBreaker, double targetDir) {
        this.manipulator = manipulator;
        this.beamBreaker = beamBreaker;
        this.targetDir = targetDir;

        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        System.out.println("manioualtor");
        
        //pivot.run(PivotConstants.L2_POSITION);
        //manipulator.runForward(targetDir);
        //manipulator.runForward(ManipulatorConstants.CORAL_SCORE_SPEED);

        if (!beamBreaker.isPressed()){
            while (!beamBreaker.isPressed()) {
                manipulator.runForward(targetDir);
            }
        } else {
            System.out.println("BEAM BREAK HIHIHI");
            manipulator.stop();
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        //elevator.stop();
        manipulator.stop();
    }

    @Override
    public boolean isFinished() {
        return beamBreaker.isPressed();
    }
}
