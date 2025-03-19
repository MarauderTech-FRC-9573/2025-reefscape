package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
    
    private TalonFX manipulator;
    private final DigitalInput beamBreaker;

    public Manipulator() {
        this.beamBreaker = new DigitalInput(0);
        manipulator = new TalonFX(ManipulatorConstants.MANIPULATOR_MOTOR_ID);
    }

    public void runForward(double forwardSpeed) {
        manipulator.set(forwardSpeed);
    }

    public void runBack(double backwardSpeed) {
        manipulator.set(backwardSpeed);
    }

    public DigitalInput getBeamBreak() { 
        return beamBreaker;
    }

    public void stop() {
        manipulator.set(0);
    }
}
