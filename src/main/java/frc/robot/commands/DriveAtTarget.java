package frc.robot.commands;

import static edu.wpi.first.units.Units.Minute;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision.Cameras;

public class DriveAtTarget extends Command {
    CommandXboxController xboxController;
    PhotonCamera photonCamera;
    SwerveSubsystem swerveDrive;
    PhotonTrackedTarget target;
    PhotonPipelineResult reading;
    int targetId;
    double yaw;
    double pitch;
    double skew;
    double yaw_min_bound = -0.1;
    double yaw_max_bound = 0.1;
    double turn;
    double targetRange;
    double drive;
    double fin_dist_min = 0.5; // No closer than this many meters
    double fin_dist_max = 0.6; // No farther than this many meters
        
        public DriveAtTarget(CommandXboxController xbox, PhotonCamera camera, SwerveSubsystem swerve) {
            xboxController = xbox;
            photonCamera = camera;
            swerveDrive = swerve;
        }
        
        @Override
        public void initialize() {}
        
        @Override
        public void execute() {
            try { 
                var unreadResults = photonCamera.getAllUnreadResults();
                if (!unreadResults.isEmpty()) { 
                    var result = unreadResults.get(unreadResults.size() - 1);
                    var target = result.getBestTarget();
                    targetId = target.getFiducialId();
    
                    //These values don't match up with the ones on the camera server/advscope
                    yaw = target.getYaw();
                    pitch = target.getPitch();
                    skew = target.getSkew();
    
                    // Yaw of Tag * kP Heading. This can be different from the one in controllerproperties.json
                    turn = yaw * 0.021 * 2.5;
                    
                    //Our distance from the target in meters
                    targetRange = 0 - PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(11.5), Units.inchesToMeters(8.5), Units.degreesToRadians(0), Units.degreesToRadians(target.getPitch()));
                    //Drive away from target if you are too close somehow.
                    if(targetRange <= fin_dist_min) {
                        drive = targetRange * -0.5;
                    }
                    else {
                        drive = targetRange * 0.5;
                    }
    
                    //drive up to 0.25 meters up to target. Robot relative.
                    swerveDrive.getSwerveDrive().drive(new Translation2d(   drive, 0), turn, false , true);
                    
                    SmartDashboard.putNumber("ID", targetId);
                    SmartDashboard.putNumber("Yaw", yaw);
                    SmartDashboard.putNumber("TargetRange", targetRange);
                }    
            } catch (Exception e) {
                System.out.println("No target found..." + e);
            }
            
        }
    
        @Override
        public boolean isFinished() {
            //Yaw to 0 is centered and range is the desired range
            if (((yaw >= yaw_min_bound) && (yaw <= yaw_max_bound)) && (targetRange >= fin_dist_min) && (targetRange <= fin_dist_max))  {
            return true;
        } else {
            return false;
        }
    }
}
