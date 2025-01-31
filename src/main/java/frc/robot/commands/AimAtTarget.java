package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
    PhotonTrackedTarget target;
    PhotonPipelineResult reading;
    
    public AimAtTarget(CommandXboxController xbox, Cameras camera, SwerveSubsystem swerve) {
        xboxController = xbox;
        photonCamera = camera;
        swerveDrive = swerve;
    }
    
    @Override
    public void initialize() {
        resultO = photonCamera.getBestResult();
        reading = resultO.get();
        System.out.println(reading.getBestTarget().getFiducialId());
    }
    
    @Override
    public void execute() {

    }
}
