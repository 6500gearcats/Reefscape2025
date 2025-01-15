package frc.team6500;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.DriveArgs;

public class Vision {
    private final PhotonCamera m_camera;
    // private final PhotonCamera cameraNote;
    private final PhotonPoseEstimator photonEstimator;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private DriveArgs driveArgs;


    public class VisionConstants {

        public static final String kCameraNameTag = "Microsoft_LifeCam_HD-3000";
        public static final String kCameraNameNote = "Microsoft_LifeCam_VX-5000";
        public static final String kCameraNameGlobal = "Global_Shutter_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(27);

        public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
        // Angle between horizontal and the camera.
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-28);

        // How far from the target we want to be
        public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  }

    public Vision(PhotonCamera camera, DriveArgs driveArgs) {
        this.driveArgs = driveArgs;

        m_camera = camera;

        photonEstimator = new PhotonPoseEstimator(
                VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (driveArgs.simulation) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(VisionConstants.kTagLayout);
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
            visionSim.addCamera(cameraSim, VisionConstants.kRobotToCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

    //TODO: Properly Implement simulation, and ask Bennet if this is correct
    public void simulationPeriodic(Pose2d pose) {
            
    }

    public Field2d getSimDebugField() {
        if (!driveArgs.simulation)
            return null;
        return visionSim.getDebugField();
    }
}
