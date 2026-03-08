// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;


import frc.robot.Robot;

public class Vision extends SubsystemBase {

  // field
  public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltAndymark);

  // Cameras
  private final PhotonCamera cameraR;
  private final PhotonCamera cameraL;

  private final Transform3d kRobotToCamR = new Transform3d(new Translation3d(-0.13172, 0.32659, 0.338),
      new Rotation3d(0, 0, 0));
  private final Transform3d kRobotToCamL = new Transform3d(new Translation3d(-0.13172, -0.32659, 0.338),
      new Rotation3d(0, 0, 0));

  private final PhotonPoseEstimator photonEstimatorR;
  private final PhotonPoseEstimator photonEstimatorL;

  private Matrix<N3, N1> curStdDevsR;
  private Matrix<N3, N1> curStdDevsL;

  private Supplier<Pose2d> currentPose;

  // private final EstimateConsumer estConsumer;

  /** Creates a new Vision. */
  public Vision() {

    cameraR = new PhotonCamera("right camera");
    cameraL = new PhotonCamera("left camera");

    photonEstimatorR = new PhotonPoseEstimator(aprilTagFieldLayout, kRobotToCamR);
    photonEstimatorL = new PhotonPoseEstimator(aprilTagFieldLayout, kRobotToCamL);

    if (Robot.isSimulation()) {

    }
  }

  public void updatePoseCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator, Transform3d cameraToRobot) {
    var result = camera.getLatestResult();

    if (!result.hasTargets())
      return;

    PhotonTrackedTarget target = result.getBestTarget();

    if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
      Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
          aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
    }
    Optional<EstimatedRobotPose> pose = poseEstimator.estimateCoprocMultiTagPose(result);

    if (pose.isEmpty()) {
      pose = poseEstimator.estimateLowestAmbiguityPose(result);
    }

    // Optional<EstimatedRobotPose> visionEst = Optional.empty();
    // for (var result : camera.getAllUnreadResults()) {
    //   visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
    //   if (visionEst.isEmpty()) {
    //     visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
    //   }
    //   updateEstimationStdDevs(visionEst, result.getTargets());

    //   if (Robot.isSimulation()) {
    //     visionEst.ifPresentOrElse(
    //         est -> getSimDebugField()
    //             .getObject("VisionEstimation")
    //             .setPose(est.estimatedPose.toPose2d()),
    //         () -> {
    //           getSimDebugField().getObject("VisionEstimation").setPoses();
    //         });
    //   }

    //   visionEst.ifPresent(
    //       est -> {
    //         // Change our trust in the measurement based on the tags we can see
    //         var estStdDevs = getEstimationStdDevs();

    //         estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    //       });
    // }
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // updatePoseCamera(cameraR, photonEstimatorR, robotToCamR);
    // updatePoseCamera(cameraL, photonEstimatorL, robotToCamL);
  }
}
