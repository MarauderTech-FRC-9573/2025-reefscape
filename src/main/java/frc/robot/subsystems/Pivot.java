package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
    private SparkMax pivot;
    
    public Pivot() {
        pivot = new SparkMax(PivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(PivotConstants.SMART_CURRENT_LIMIT);
        pivot.configure(pivotConfig, null, null);
    }

    public void resetEncoders() {
        pivot.getEncoder().setPosition(0);
    }

    public void run(double position) {
        if (pivot.getEncoder().getPosition() > position) {
            while (pivot.getEncoder().getPosition() != position) {
                pivot.set(PivotConstants.PIVOT_SPEED_DOWN);
            }
        } else if (pivot.getEncoder().getPosition() < position) {
            while (pivot.getEncoder().getPosition() != position) {
                pivot.set(PivotConstants.PIVOT_SPEED_UP);
            }
        } 
        this.stop();
    }
    
    public void runUp() {
        pivot.set(PivotConstants.PIVOT_SPEED_UP);
    }
    public void runDown() {
        pivot.set(PivotConstants.PIVOT_SPEED_DOWN);
    }

    public void stop() {
        pivot.set(0.1);
    }
}
