
package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveSubsystem;

public class GCLimelight {
    // Limelight name
    private String name;

    public GCLimelight(String theName) {
        name = theName;
    }

    public double getYawDegrees() {
        if (LimelightHelpers.getTV(name)) {
            return LimelightHelpers.getTX(name);
        }
        return 0;
    }

    public double getPitchDegrees() {
        if (LimelightHelpers.getTV(name)) {
            return LimelightHelpers.getTY(name);
        }
        return 0;
    }

    public double getTargetAreaPercent() {
        if (LimelightHelpers.getTV(name)) {
            return LimelightHelpers.getTA(name);
        }
        return 0;
    }

    public double getTargetDistanceX() {
        double ta = LimelightHelpers.getTA(name);
        if (LimelightHelpers.getTV(name)) {
            return 38 * Math.sqrt(3.77 / ta);
        } else {
            return 0;
        }

        // Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace(name);
        // double verticalOffset = LimelightHelpers.getTY(name);
        // double height = pose.getZ() - verticalOffset;
    }

    public double getChosenTargetYawDegrees(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (LimelightHelpers.getTV(name)) {
            for (int i = 0; i < results.targets_Fiducials.length; i++) {
                if (results.targets_Fiducials[i].fiducialID == fiducialID) {
                    return results.targets_Fiducials[i].tx;
                }
            }
        }
        return 0.0;
    }

    public double getChosenTargetPitchDegrees(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (LimelightHelpers.getTV(name)) {
            for (int i = 0; i < results.targets_Fiducials.length; i++) {
                if (results.targets_Fiducials[i].fiducialID == fiducialID) {
                    return results.targets_Fiducials[i].ty;
                }
            }
        }
        return 0.0;
    }

    public double getBestSkewDegrees() {
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(name);

        if (LimelightHelpers.getTV(name)) {
            return targetPose.getRotation().getZ();
        } else {
            return 0.0;
        }
    }

    public double getChosenTargetSkewDegrees(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (LimelightHelpers.getTV(name)) {
            for (int i = 0; i < results.targets_Fiducials.length; i++) {
                if (results.targets_Fiducials[i].fiducialID == fiducialID) {
                    return results.targets_Fiducials[i].ts;
                }
            }
        }
        return 0.0;
    }

    public double getChosenTargetDistanceX(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (LimelightHelpers.getTV(name)) {
            for (int i = 0; i < results.targets_Fiducials.length; i++) {
                if (results.targets_Fiducials[i].fiducialID == fiducialID) {
                    double x = (6.5 / (2 * getTargetAreaPercent())) / Math.tan(90 * Math.PI / 180);
                    double distance = x / Math.cos(getChosenTargetYawDegrees(fiducialID) * Math.PI / 180);
                    return distance;
                }
            }
        }
        return 0.0;
    }
}
