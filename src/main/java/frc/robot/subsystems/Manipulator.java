package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
    
    private TalonFX manipulator;
    private final SparkLimitSwitch beamBreaker;

    public Manipulator() {
        SparkMax pivotMotor = new SparkMax(16, MotorType.kBrushless);
        this.beamBreaker = pivotMotor.getForwardLimitSwitch();
        manipulator = new TalonFX(ManipulatorConstants.MANIPULATOR_MOTOR_ID);
    }

    public void runForward(double forwardSpeed) {
        manipulator.set(forwardSpeed);
    }

    public void runBack(double backwardSpeed) {
        manipulator.set(backwardSpeed);

        if (manipulator.getMotorStallCurrent().getValueAsDouble() > 30) {
            this.stop();
        }
    }

    public SparkLimitSwitch getBeamBreak() { 
        return beamBreaker;
    }

    public void stop() {
        manipulator.set(0);
    }
}
