package frc.robot.subsystems;

import org.photonvision.PhotonUtils;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Vision.Cameras;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Transform3d;
import swervelib.SwerveDrive;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.photonvision.targeting.MultiTargetPNPResult;
import java.awt.Desktop;
import java.util.List;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import edu.wpi.first.networktables.NetworkTablesJNI;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;


public class Vision {
    //Apriltag layout of the year
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2025Reefscape);
    //Ambiguityt is value between (0,1)
    private final double maximumAmbiguity = 0.25; 
    //Count of times it thinks we're 10m away from the apriltag
    private double longDistancePoseEstimationCount = 0;
    //Current pose from pose estimator using wheel odometry
    private Supplier<Pose2d> currentPose;
    //field
    private Field2d field2d;
    public Vision(Supplier<Pose2d> currentPose, Field2d field){
        this.currentPose = currentPose;
        this.field2d = field;
    }
    //Calculates the target pose of the april tag
    public static Pose2d getAprilTagPose(int apriltag, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(apriltag);
        if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException("Cannot get AprilTag" + apriltag +" from field" + fieldLayout.toString());
        }
    }
    //Gets the lastest result
    public void updatePoseEstimation (SwerveDrive swerveDrive){
        for (Cameras camera : Cameras.values()) {
            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
            if (poseEst.isPresent()) {
                var pose = poseEst.get();
                swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), 
                                                 pose.timestampSeconds,
                                                 camera.curStdDevs);
            }
        }
    }
    // Generates estimated robot pose
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
        Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
        return poseEst;
    }

    //Get distance from AprilTag
    public double getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
    }

    //Get tracked target from camera of AprilTagID
    public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
        PhotonTrackedTarget target = null;
        for(PhotonPipelineResult result : camera.resultsList) {
            if (result.hasTargets()) {
                for (PhotonTrackedTarget i : result.getTargets()) {
                    if(i.getFiducialId() == id) {
                        return i;
                    }
                }
            }
        }

        return target;
    }

    //Open up the phtonvision camera streams on the localhost, assumes running photon vision on local host.
    private void openSimCameraViews() {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            try {
                Desktop.getDesktop().browse(new URI("https/localhost:1182/"));
                Desktop.getDesktop().browse(new URI("https/localhost:1184/"));
                Desktop.getDesktop().browse(new URI("https/localhost:1186/"));
            } catch (IOException | URISyntaxException e) {
                e.printStackTrace();
            }
        }
    }

    //Update Field to include tracked Targets
    public void updateVisionField() {
        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
        for(Cameras c : Cameras.values()) {
            if(!c.resultsList.isEmpty()) {
                PhotonPipelineResult latest = c.resultsList.get(0);
                if (latest.hasTargets()) {
                    targets.addAll(latest.targets);
                }
            }
        }

        List<Pose2d> poses = new ArrayList<>();
        for(PhotonTrackedTarget target : targets) {
            if(fieldLayout.getTagPose(target.getFiducialId()).isPresent()){
                Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
                poses.add(targetPose);
            }
        }
        field2d.getObject("tracked targets").setPoses(poses);
    }

    /*Camera Enum to select each Camera
    Need to add specifics
     */
        enum Cameras {
            CenterCam("Marlin",
                    new Rotation3d(0, Units.degreesToRadians(0), 0), 
                    new Translation3d(Units.inchesToMeters(0), 
                                      Units.inchesToMeters(0), 
                                      Units.inchesToMeters(0)), 
                        VecBuilder.fill(0, 0, 0), VecBuilder.fill(0, 0, 0));
    }

    //Latency alert from whn high latency is detected
    public final Alert latencyAlert;

    //Camera instance for comms
    public final PhotonCamera camera;

    //Pose Estimator for camera
    public final PhotonPoseEstimator poseEstimator;

    //Current standard deviations used
    public Matrix<N3, N1> curStdDevs;

    //Estimated Robot Pose
    public Optional<EstimatedRobotPose> estimatedRobotPose;

    //Result list to be updated periodically
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    //Last read from the camera timestamp to prevent lag
    private double lastReadTimestamp = Microsecond.of(NetworkTablesJNI.now()).in(Second);

    //Transform of the camera rotation and translation relative to the v=center of the robot
    private final Transform3d robotToCamTransform;

    //Standar deviation for single tag readings
    public Matrix<N3, N1> singleTagStdDevs;

    //Standar deviation for multiple tag readings
    public Matrix<N3, N1> multiTagStdDevs;

    //Camera Class
    public void Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
        latencyAlert = new Alert("'" + name + "' is experiencing high latency.", AlertType.kWarning);
        camera = new PhotonCamera(name);
        robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
        poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, 
                                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                                                robotToCamTransform);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.singleTagStdDevs = singleTagStdDevs;
        this.multiTagStdDevs = multiTagStdDevs;


    }

    //The result in the cache with the least ambiguous best tracked target. Not the most recent
    public Optional<PhotonPipelineResult> getBestResult() {
     if (resultsList.isEmpty())
     {
          return Optional.empty();
      }
      PhotonPipelineResult bestResult       = resultsList.get(0);
      double               amiguity         = bestResult.getBestTarget().getPoseAmbiguity();
     double               currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList)
        {
         currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
         if (currentAmbiguity < amiguity && currentAmbiguity > 0)
         {
            bestResult = result;
            amiguity = currentAmbiguity;
          }
         }
         return Optional.of(bestResult);
        } 
    /** Get the latest result from the current cache.
    @return Empty optional if nothing is found. Latest result if something is there.
    */
   public Optional<PhotonPipelineResult> getLatestResult() {
    return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
   }

    /**Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
    cache of results.
    @return Estimated pose */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose()
    {
        updateUnreadResults();
        return estimatedRobotPose;
    }

     /** Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp.*/
    private void updateUnreadResults()
    {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
     double currentTimestamp    = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
     double debounceTime        = Milliseconds.of(15).in(Seconds);
        for (PhotonPipelineResult result : resultsList)
     {
       mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
     }
         if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
             (currentTimestamp - lastReadTimestamp) >= debounceTime)
     {
     resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
     lastReadTimestamp = currentTimestamp;
     resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
           return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
     });
      if (!resultsList.isEmpty())
         {
          updateEstimatedGlobalPose();
         }
      }
    }
    /**The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
    per loop. 
    <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
    {@link Cameras#updateEstimationStdDevs}
    @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for
      estimation. */

    private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

/** The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
    per loop.
    <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
    {@link Cameras#updateEstimationStdDevs}
    @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for
    estimation.*/

    private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
      Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
      on number of tags, estimation strategy, and distance from the tags.
      @param estimatedPose The estimated pose to guess standard deviations for.
      @param targets       All targets in this camera frame*/
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
    {
      if (estimatedPose.isEmpty())
      {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else
      {
        // Pose present. Start running Heuristic
        var    estStdDevs = singleTagStdDevs;
        int    numTags    = 0;
        double avgDist    = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets)
        {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty())
          {
            continue;
          }
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0)
        {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else
        {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1)
          {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
          {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else
          {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      } 
    }
  }
