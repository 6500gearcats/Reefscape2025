/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 import static frc.robot.Constants.VisionConstants.*;
 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.math.util.Units;
 import edu.wpi.first.wpilibj.smartdashboard.Field2d;
 import frc.robot.Constants.VisionConstants;
 
 import java.util.List;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.PhotonUtils;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonPipelineResult;
 import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class GCPhotonVision {
     private final PhotonCamera m_camera;
     // private final PhotonCamera cameraNote;
     private final PhotonPoseEstimator photonEstimator;
     private double lastEstTimestamp = 0;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;
 
     public GCPhotonVision(PhotonCamera camera) {
         m_camera = camera;
 
         photonEstimator = new PhotonPoseEstimator(
                 kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
         // ----- Simulation
         if (Robot.isSimulation()) {
             // Create the vision system simulation which handles cameras and targets on the
             // field.
             visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this
             // simulated field.
             visionSim.addAprilTags(kTagLayout);
             // Create simulated camera properties. These can be set to mimic your actual
             // camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values
             // with visible
             // targets.
             cameraSim = new PhotonCameraSim(m_camera, cameraProp);
             // Add the simulated camera to view the targets on this simulated field.
             visionSim.addCamera(cameraSim, kRobotToCam);
 
             cameraSim.enableDrawWireframe(true);
         }
     }
 
     public PhotonPipelineResult getLatestResult() {
         return m_camera.getLatestResult();
     }
 
     public PhotonPipelineResult getLatestCameraResult() {
         return m_camera.getLatestResult();
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be
      * empty. This should
      * only be called once per loop.
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
      *         timestamp, and targets
      *         used for estimation.
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         var visionEst = photonEstimator.update(getLatestCameraResult());
         double latestTimestamp = m_camera.getLatestResult().getTimestampSeconds();
         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
         if (Robot.isSimulation()) {
             visionEst.ifPresentOrElse(
                     est -> getSimDebugField()
                             .getObject("VisionEstimation")
                             .setPose(est.estimatedPose.toPose2d()),
                     () -> {
                         if (newResult)
                             getSimDebugField().getObject("VisionEstimation").setPoses();
                     });
         }
         if (newResult)
             lastEstTimestamp = latestTimestamp;
         return visionEst;
     }
 
     /**
      * The standard deviations of the estimated pose from
      * {@link #getEstimatedGlobalPose()}, for use
      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
      * SwerveDrivePoseEstimator}.
      * This should only be used when there are targets visible.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      */
     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
         var estStdDevs = kSingleTagStdDevs;
         var targets = getLatestResult().getTargets();
         int numTags = 0;
         double avgDist = 0;
         for (var tgt : targets) {
             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
             if (tagPose.isEmpty())
                 continue;
             numTags++;
             avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
         }
         if (numTags == 0)
             return estStdDevs;
         avgDist /= numTags;
         // Decrease std devs if multiple targets are visible
         if (numTags > 1)
             estStdDevs = kMultiTagStdDevs;
         // Increase std devs based on (average) distance
         if (numTags == 1 && avgDist > 4)
             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         else
             estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
 
         return estStdDevs;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation())
             visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation())
             return null;
         return visionSim.getDebugField();
     }

     public double getBestTargetRotation()
     {
        var result = getLatestCameraResult();

        if(result.hasTargets()){
            return result.getBestTarget().getYaw();
        }

        return 0;
     }

     public double getBestTargetRange()
     {
        var result = getLatestCameraResult();

        if(result.hasTargets()){
            return PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS, // Previously declarde
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return 0;
     }

     public double getBestTargetPitch()
     {
        var result = getLatestCameraResult();

        if(result.hasTargets()){
            return result.getBestTarget().getPitch();
        }

        return 0.0;
     }
 
     public double getChosenTargetRotation(int targetID) {
         var result = getLatestCameraResult();
         // Get a list of all of the targets that have been detected.
         List<PhotonTrackedTarget> targets = result.getTargets();
         double rotation = 0;
 
         // For each target we have check if it matches the id you want.
         for (PhotonTrackedTarget target : targets) {
             if (result.hasTargets()) {
                 if (target.getFiducialId() == targetID) {
                     // Use the value of target to find our rotation using the getYaw command
                     return target.getYaw();
                 }
             } else {
                 rotation = 0;
             }
         }
 
         return rotation;
     }

     public double getChosenTargetPitch(int targetID) {
        var result = getLatestCameraResult();
        // Get a list of all of the targets that have been detected.
        List<PhotonTrackedTarget> targets = result.getTargets();
        double rotation = 0;

        // For each target we have check if it matches the id you want.
        for (PhotonTrackedTarget target : targets) {
            if (result.hasTargets()) {
                if (target.getFiducialId() == targetID) {
                    // Use the value of target to find our rotation using the getYaw command
                    return target.getPitch();
                }
            } else {
                rotation = 0;
            }
        }

        return rotation;
    }

    public double getChosenTargetSkew(int targetID) {
        var result = getLatestCameraResult();
        // Get a list of all of the targets that have been detected.
        List<PhotonTrackedTarget> targets = result.getTargets();
        double rotation = 0;

        // For each target we have check if it matches the id you want.
        for (PhotonTrackedTarget target : targets) {
            if (result.hasTargets()) {
                if (target.getFiducialId() == targetID) {
                    // Use the value of target to find our rotation using the getYaw command
                    return target.getSkew();
                }
            } else {
                rotation = 0;
            }
        }

        return rotation;
    }

    public boolean hasChosenTarget(int targetID) {
        var result = getLatestCameraResult();
        // Get a list of all of the targets that have been detected.
        List<PhotonTrackedTarget> targets = result.getTargets();

        // For each target we have check if it matches the id you want.
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == targetID) {
                // Use the value of target to find our rotation using the getYaw command
                return true;
            }
        }

        return false;
    }

     public double getChosenTargetRange(int targetID) {
         var result = getLatestCameraResult();
         List<PhotonTrackedTarget> targets = result.getTargets();
         double range = 0;
         if (result.hasTargets()) {
             for (PhotonTrackedTarget target : targets) {
                 if (target.getFiducialId() == targetID) {
                     range = PhotonUtils.calculateDistanceToTargetMeters(
                             CAMERA_HEIGHT_METERS, // Previously declarde
                             TARGET_HEIGHT_METERS,
                             CAMERA_PITCH_RADIANS,
                             Units.degreesToRadians(target.getPitch()));
                     return range;
                 }
             }
         }
         return 0;
     }
 
     public double getRange() {
         var result = getLatestResult();
         double range;
         if (result.hasTargets()) {
             // First calculate range
             range = PhotonUtils.calculateDistanceToTargetMeters(
                     CAMERA_HEIGHT_METERS, // Previously declarde
                     TARGET_HEIGHT_METERS,
                     CAMERA_PITCH_RADIANS,
                     Units.degreesToRadians(result.getBestTarget().getPitch()));
             return range;
 
         } else {
             // If we have no targets, stay still.
             return 0;
             // When this is implemented - DO NOTHING IF RANGE IS 0
         }
     }
 
     public double getYaw() {
         var result = getLatestCameraResult();
         double yaw = 0.0;
         if (result.hasTargets()) {
             // Calculate angular turn power
             // Remove -1.0 because it was inverting results.
             yaw = result.getBestTarget().getYaw();
         }
         return yaw;
     }
 
     public double getPitch() {
         var result = getLatestCameraResult();
         double pitch = 0.0;
         if (result.hasTargets()) {
             // Calculate angular turn power
             // Remove -1.0 because it was inverting results.
             pitch = result.getBestTarget().getPitch();
         }
 
         return pitch;
     }
 
 }
 