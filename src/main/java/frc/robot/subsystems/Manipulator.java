package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
    
    // private SparkMax manipulator; // Formerly a Talon, changed to be NEO to debug manipulator :( 
    private TalonFX manipulator;

    public Manipulator() {
        manipulator = new TalonFX(17);
        // manipulator = new SparkMax(17, MotorType.kBrushless);
    }

    public void runForward(double forwardSpeed) {
            manipulator.set(forwardSpeed);
    }

    public void runBack(double backwardSpeed) {
        manipulator.set(-backwardSpeed);

        // if (manipulator.getOutputCurrent() > ManipulatorConstants.MANIPULATOR_BACKWARD_STOP) {
        //     this.stop();
        // }

        // if (manipulator.getMotorStallCurrent().getValueAsDouble() > ManipulatorConstants.MANIPULATOR_BACKWARD_STOP) {
        //     this.stop();
        // }
    }


    public void stop() {
        manipulator.set(ManipulatorConstants.MANIPULATOR_STOP);
    }
}
