package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision.Cameras;

public class AimAtTarget extends Command {
    CommandXboxController xboxController;
    Cameras photonCamera;
    SwerveSubsystem swerveDrive;
    Optional<PhotonPipelineResult> resultO;
    
    public AimAtTarget(CommandXboxController xbox, Cameras camera, SwerveSubsystem swerve) {
        xboxController = xbox;
        photonCamera = camera;
        swerveDrive = swerve;
    }
    
    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        resultO = photonCamera.getBestResult();
        if (resultO.isPresent()) {
            try { 
                var result = resultO.get();
                if (result.hasTargets())
                {
                    SmartDashboard.putNumber("ID Recognized", result.getBestTarget().getFiducialId());
                    double turn = -1.0 * result.getBestTarget().getYaw() * 0.021 * 0.3;
                    System.out.println("ID" + result.getBestTarget().getFiducialId());
                    System.out.println("Yaw" + result.getBestTarget().getYaw());
                    System.out.println("Turn Amt" + turn);
                    swerveDrive.getSwerveDrive().drive(new Translation2d(0, 0), Math.toDegrees(turn), isScheduled(), isFinished());
                }  
        } catch (Exception e) {
            System.out.println("No target found..." + e);
        }
    }
    }
}
