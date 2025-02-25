package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
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
    PhotonCamera photonCamera;
    SwerveSubsystem swerveDrive;
    PhotonTrackedTarget target;
    PhotonPipelineResult reading;
    int targetId;
    double yaw;
    double turn;
    double initIMU;
    
    public AimAtTarget(CommandXboxController xbox, PhotonCamera camera, SwerveSubsystem swerve) {
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
            var unreadResults = photonCamera.getAllUnreadResults();
            if (!unreadResults.isEmpty()) { 
                var result = unreadResults.get(unreadResults.size() - 1);
                var target = result.getBestTarget();
                targetId = target.getFiducialId();
                yaw = target.getYaw();
                turn = yaw * 0.021 * 2.5;
    
                swerveDrive.getSwerveDrive().drive(new Translation2d(0, 0), turn, isScheduled(), isFinished());
    
                SmartDashboard.putNumber("ID", targetId);
                SmartDashboard.putNumber("Yaw", yaw);
                SmartDashboard.putNumber("Turn", turn);
            }    
        } catch (Exception e) {
            System.out.println("No target found..." + e);
        }
        
    }
}
