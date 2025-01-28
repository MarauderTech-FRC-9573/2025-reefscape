package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision.Cameras;
import swervelib.SwerveDrive;

public class AimAtTarget extends Command {
    CommandXboxController xboxController;
    Cameras photonCamera;
    SwerveSubsystem swerveDrive;
    
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
        try { 
            System.out.println("Pipeline getting results");
            Optional<PhotonPipelineResult> resultO = photonCamera.getBestResult();
            if (resultO.isPresent())
            {
                var result = resultO.get();
                if (result.hasTargets())
                {
                    SmartDashboard.putNumber("ID Recognized", result.getBestTarget().getFiducialId());
                    double turn = -1.0 * result.getBestTarget().getYaw() * 0.021 * 0.3;
                    SmartDashboard.putNumber("Turn Amt", turn);
                    swerveDrive.getSwerveDrive().drive(new Translation2d(0, 0), Math.toDegrees(turn), isScheduled(), isFinished());
                }  
            } 
        } catch (Exception e) {
            SmartDashboard.putNumber("ID Recognized", -9999);
        }

    }
}
