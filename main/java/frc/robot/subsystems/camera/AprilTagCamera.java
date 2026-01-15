package frc.robot.subsystems.camera;

import java.util.Optional;

import frc.robot.Constants;
import org.jcp.xml.dsig.internal.dom.Utils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagCamera extends SubsystemBase {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final SendableChooser<Boolean> cameraEnabled;

    public AprilTagCamera(String cameraName, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(cameraName);
        aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        this.poseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera
        );

        cameraEnabled = new SendableChooser<Boolean>();
        cameraEnabled.setDefaultOption("Enabled", true);
        cameraEnabled.addOption("Disabled", false);
        SmartDashboard.putData(String.format("%s Enabled?", cameraName), cameraEnabled);
    }

    @Override
    public void periodic() {

        var results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {   

            SmartDashboard.putBoolean("AprilTag Detected", result.hasTargets());

            boolean hasPose = false;

            if (result.hasTargets()) {
                SmartDashboard.putNumber("Target Count", result.targets.size());

                for (PhotonTrackedTarget target : result.targets) {
                    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                        for (int tag : Constants.AutoDriveConstants.GOOD_BLUE_APRIL_TAGS) {
                            if (target.getFiducialId() == tag) {
                                hasPose = true;
                                break;
                            }
                        }
                    } else {
                        for (int tag : Constants.AutoDriveConstants.GOOD_RED_APRIL_TAGS) {
                            if (target.getFiducialId() == tag) {
                                hasPose = true;
                                break;
                            }
                        }
                    }
                    
                    if (hasPose) {
                        break;
                    }
                }

                if (hasPose) {
                    // Pose estimation
                    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);

                    if (estimatedPose.isPresent()) {
                        Pose3d visionPose = estimatedPose.get().estimatedPose;
                        if (this.cameraEnabled.getSelected()) {
                            drivetrain.addVisionMeasurement(visionPose.toPose2d(), Utils.getCurrentTimeSeconds());
                        }
                        SmartDashboard.putString("Vision Pose", visionPose.toString());
                    }
                }
            } else {
            }
        }
    }
}