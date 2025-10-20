package frc.robot.commands.ManipulatorCommands;

import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;



public class ManipulatorAutoCommand extends Command{
    private final Manipulator manipulator;
    public static double seconds = 0.5;
    private final Double speed;

    private final Timer timer = new Timer();

    public ManipulatorAutoCommand(Manipulator manipulator) {
        this.manipulator = manipulator;
        this.speed = 1.0;
        addRequirements(manipulator);
        }

        @Override 
        public void initialize() {

            timer.reset();
            timer.start();
            manipulator.manualControl(speed);

        }


        @Override
        public void end(boolean interrupted) {
            manipulator.manualControl(0); // Stop the manipulator when the command ends
        }
    
        @Override 
        public boolean isFinished() {
            return timer.hasElapsed(seconds);
        }
}
