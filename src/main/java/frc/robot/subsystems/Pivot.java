package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private SparkMax pivot;
    private final SparkLimitSwitch beamBreaker;
    
    public Pivot() {
        pivot = new SparkMax(16, MotorType.kBrushless);
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(PivotConstants.SMART_CURRENT_LIMIT);
        pivot.configure(pivotConfig, null, null);
        this.beamBreaker = pivot.getForwardLimitSwitch();
        resetEncoders();
    }

    public void resetEncoders() {
        pivot.getEncoder().setPosition(0);
    }

    public void run(double position) {
        if (pivot.getEncoder().getPosition() < position) {
            while (pivot.getEncoder().getPosition() < position) {
                runUp();
            }
        } else if (pivot.getEncoder().getPosition() > position) {
            while (pivot.getEncoder().getPosition() > position) {
                runDown();
            }
        } else if (pivot.getEncoder().getPosition() == position) {
            stop();
        }
        stop();
    }
    
    public void runUp() {

        // if (pivot.getOutputCurrent() > 30) {
        //     this.resetEncoders();
        // }

        if (pivot.getEncoder().getPosition() >= PivotConstants.MAX_RETRACTION) {
            this.stop();
        } else {
            pivot.set(PivotConstants.PIVOT_SPEED_UP);
        }   
    }
    public void runDown() {

        if (pivot.getEncoder().getPosition() <= PivotConstants.MAX_EXTENTION) {
                this.stop();
            } else{
            pivot.set(PivotConstants.PIVOT_SPEED_DOWN);
        }   
    }

    public SparkLimitSwitch getBeamBreak() { 
        return beamBreaker;
    }

    @Override
    public void periodic() {
        // System.out.println("Current" + pivot.getOutputCurrent());
        // System.out.println("Encoder" + pivot.getEncoder().getPosition());

    }

    public void stop() {
        if (pivot.getEncoder().getPosition() >= -9.0) { 
            pivot.set(PivotConstants.PIVOT_STOP_BSPEED);
        } else {
            pivot.set(PivotConstants.PIVOT_STOP_FSPEED);
        }
    }
}
