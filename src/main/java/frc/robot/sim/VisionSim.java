// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;
import limelight.Limelight;
import limelight.sim.LimelightSim;
import limelight.sim.LimelightSimSettings;

/**
 * Simulates the robot's three Limelights with YALL's {@link LimelightSim}.
 *
 * <p>Every loop each camera is handed the ground-truth robot pose (with bump tilt), works out which
 * AprilTags it can see, and writes the same NetworkTables keys a real Limelight would ({@code tv},
 * {@code botpose_orb_wpiblue}, {@code rawfiducials}, {@code json}, ...). So the normal vision code
 * ({@code Vision} + {@code StateMachine.setRobotPose} via {@link LimelightHelpers}) runs unchanged.
 * MegaTag2 uses the yaw the robot sends in {@code robot_orientation_set}, like the real camera.
 *
 * <p>The simulated camera does not render images or solve PnP: it reports the true pose plus
 * Gaussian noise that grows with tag distance and shrinks with tag count. Tags are not blocked by
 * field elements or other robots.
 *
 * <h2>What you can look at in AdvantageScope</h2>
 * <ul>
 *   <li><b>NT "Sim/Vision/&lt;camera&gt;/VisibleTags"</b> — Pose3d[] of the tags that camera sees.
 *       Drag onto the robot in a 3D Field as <b>Vision Target</b> to draw lines to them.
 *       "Sim/Vision/VisibleTags" has every camera's tags in one array.
 *   <li><b>NT "Sim/Vision/&lt;camera&gt;/EstimatedPose"</b> — Pose2d[] (0 or 1) with that camera's
 *       MegaTag2 pose, read back through LimelightHelpers. Add as a <b>Ghost</b> to compare it
 *       against the true pose.
 *   <li><b>NT "Sim/Vision/&lt;camera&gt;/CameraPose"</b> — true field pose of the lens.
 *   <li><b>Camera views</b> — the "9410 2026 Robot" model's config.json defines all three cameras,
 *       so the 3D Field's camera menu can look out of each one.
 * </ul>
 *
 * <h2>Odometry</h2>
 * With <b>SmartDashboard "Sim/Vision/FuseIntoOdometry"</b> on (default), the robot's pose estimate
 * runs like the real robot: wheel odometry + gyro, corrected by these vision measurements, so it
 * drifts on the bumps and vision pulls it back. Turn it off to snap odometry to the ground-truth
 * pose every loop and ignore vision (the old behavior).
 */
public class VisionSim {
  /** SmartDashboard toggle: fuse simulated vision into odometry instead of syncing to truth. */
  public static final String FUSE_KEY = "Sim/Vision/FuseIntoOdometry";
  private static final boolean FUSE_DEFAULT = true;

  /** Farthest a tag can be detected from. YALL default; real LL4s see tags a bit farther. */
  private static final double MAX_TAG_RANGE_M = 6.0;

  /**
   * MegaTag2 translation noise (m) for one tag at 1 m; YALL scales it by (1 + distance) / sqrt(tags)
   * and halves it for MegaTag2. 0.01 gives ~2 cm at 3 m, about what a real LL4 shows. MegaTag1 yaw
   * noise (deg) only affects botpose / botpose_wpiblue, which the robot doesn't use.
   */
  private static final double POSE_NOISE_M = 0.01;
  private static final double YAW_NOISE_DEG = 1.5;

  private static final AprilTagFieldLayout FIELD =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private final List<Camera> cameras = new ArrayList<>();

  private final StructArrayPublisher<Pose3d> allVisibleTagsPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Sim/Vision/VisibleTags", Pose3d.struct)
          .publish();

  public VisionSim() {
    SmartDashboard.setDefaultBoolean(FUSE_KEY, FUSE_DEFAULT);
    addCamera("Shooter", VisionConstants.SHOOTER_TABLE, VisionConstants.ROBOT_TO_SHOOTER_CAMERA);
    addCamera("Left", VisionConstants.LEFT_TABLE, VisionConstants.ROBOT_TO_LEFT_CAMERA);
    addCamera("Right", VisionConstants.RIGHT_TABLE, VisionConstants.ROBOT_TO_RIGHT_CAMERA);
  }

  /** True when simulated vision should drive the pose estimate (see class docs). */
  public static boolean isFusionEnabled() {
    return SmartDashboard.getBoolean(FUSE_KEY, FUSE_DEFAULT);
  }

  /**
   * Publishes this loop's simulated Limelight data. Call from {@code simulationPeriodic()} after the
   * arena and bump sim have stepped, so the cameras see where the robot actually is.
   *
   * @param robotPose ground-truth robot pose, including bump height and tilt
   */
  public void update(Pose3d robotPose) {
    List<Pose3d> allVisible = new ArrayList<>();
    for (Camera camera : cameras) {
      camera.sim.update(robotPose);
      camera.cameraPosePublisher.set(robotPose.transformBy(camera.robotToCamera));

      Pose3d[] visible = visibleTags(camera.table);
      camera.visibleTagsPublisher.set(visible);
      allVisible.addAll(List.of(visible));

      LimelightHelpers.PoseEstimate mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera.table);
      camera.estimatedPosePublisher.set(
          mt2 != null && mt2.tagCount > 0 ? new Pose2d[] {mt2.pose} : new Pose2d[0]);
    }
    allVisibleTagsPublisher.set(allVisible.toArray(Pose3d[]::new));
  }

  private void addCamera(String name, String table, Transform3d robotToCamera) {
    LimelightSimSettings settings =
        new LimelightSimSettings()
            .withResolution(VisionConstants.LL4_RESOLUTION_WIDTH, VisionConstants.LL4_RESOLUTION_HEIGHT)
            .withFOV(VisionConstants.LL4_HORIZONTAL_FOV_DEG, VisionConstants.LL4_VERTICAL_FOV_DEG)
            .withMaxDetectionRange(MAX_TAG_RANGE_M)
            .withPoseNoise(POSE_NOISE_M, YAW_NOISE_DEG)
            // Separate seeds so the cameras' noise isn't identical.
            .withRandomSeed(9410L + cameras.size());
    LimelightSim sim =
        new LimelightSim(new Limelight(table), settings).withRobotToCameraTransform(robotToCamera);
    cameras.add(new Camera(name, table, robotToCamera, sim));
  }

  /** Field poses of the tags a camera just reported, read back from its "rawfiducials" entry. */
  private static Pose3d[] visibleTags(String table) {
    List<Pose3d> tags = new ArrayList<>();
    for (LimelightHelpers.RawFiducial fiducial : LimelightHelpers.getRawFiducials(table)) {
      FIELD.getTagPose(fiducial.id).ifPresent(tags::add);
    }
    return tags.toArray(Pose3d[]::new);
  }

  private static final class Camera {
    final String table;
    final Transform3d robotToCamera;
    final LimelightSim sim;
    final StructPublisher<Pose3d> cameraPosePublisher;
    final StructArrayPublisher<Pose3d> visibleTagsPublisher;
    final StructArrayPublisher<Pose2d> estimatedPosePublisher;

    Camera(String name, String table, Transform3d robotToCamera, LimelightSim sim) {
      this.table = table;
      this.robotToCamera = robotToCamera;
      this.sim = sim;
      var nt = NetworkTableInstance.getDefault();
      String prefix = "Sim/Vision/" + name + "/";
      cameraPosePublisher = nt.getStructTopic(prefix + "CameraPose", Pose3d.struct).publish();
      visibleTagsPublisher =
          nt.getStructArrayTopic(prefix + "VisibleTags", Pose3d.struct).publish();
      estimatedPosePublisher =
          nt.getStructArrayTopic(prefix + "EstimatedPose", Pose2d.struct).publish();
    }
  }
}
