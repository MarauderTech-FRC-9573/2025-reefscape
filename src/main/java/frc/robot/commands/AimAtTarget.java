package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision.Cameras;

public class AimAtTarget extends Command {
    CommandXboxController xboxController;
    PhotonCamera photonCamera;
    SwerveSubsystem swerveDrive;
    PhotonTrackedTarget target;
    PhotonPipelineResult reading;
    int targetId;
    
    public AimAtTarget(CommandXboxController xbox, PhotonCamera camera, SwerveSubsystem swerve) {
        xboxController = xbox;
        photonCamera = camera;
        swerveDrive = swerve;
    }
    
    @Override
    public void initialize() {
        var unreadResults = photonCamera.getAllUnreadResults();
        if (!unreadResults.isEmpty()) { 
            var result = unreadResults.get(unreadResults.size() - 1);
            for (var target : result.getTargets()) {
                targetId = target.getFiducialId();
                System.out.println(targetId);
            }

        }
    }

    
    @Override
    public void execute() {

    }
}
