// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Publishes and animates the currently selected PathPlanner autonomous routine. */
public class AutoPreview {
  private static final double ANIMATION_SPEED_METERS_PER_SEC = 2.0;

  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d field = new Field2d();
  private String lastPreviewedAuto = "";
  private boolean lastPreviewWasFlipped = false;
  private List<Pose2d> previewPoses = List.of();
  private double animationStartTime = 0.0;

  /** Creates a dashboard preview for the supplied autonomous chooser. */
  public AutoPreview(LoggedDashboardChooser<Command> autoChooser) {
    this.autoChooser = autoChooser;
    SmartDashboard.putData("Auto Preview", field);

    PathPlannerLogging.setLogCurrentPoseCallback(field::setRobotPose);
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> field.getObject("target pose").setPose(pose));
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    updatePreview();
  }

  /** Updates the selected route and advances its dashboard animation. Call while disabled. */
  public void periodic() {
    updatePreview();
    updateAnimation();
  }

  private void updatePreview() {
    String selectedAuto = autoChooser.getSendableChooser().getSelected();
    boolean shouldFlip = AutoBuilder.shouldFlip();

    if (selectedAuto == null || selectedAuto.equals("None")) {
      field.getObject("path").setPoses(List.of());
      previewPoses = List.of();
      lastPreviewedAuto = "";
      lastPreviewWasFlipped = shouldFlip;
      return;
    }

    if (selectedAuto.equals(lastPreviewedAuto) && shouldFlip == lastPreviewWasFlipped) {
      return;
    }

    try {
      List<Pose2d> newPreviewPoses = new ArrayList<>();
      for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(selectedAuto)) {
        PathPlannerPath previewPath = shouldFlip && !path.preventFlipping ? path.flipPath() : path;
        newPreviewPoses.addAll(previewPath.getPathPoses());
      }

      field.getObject("path").setPoses(newPreviewPoses);
      if (!newPreviewPoses.isEmpty()) {
        field.setRobotPose(newPreviewPoses.get(0));
      }
      previewPoses = List.copyOf(newPreviewPoses);
      animationStartTime = Timer.getFPGATimestamp();
      lastPreviewedAuto = selectedAuto;
      lastPreviewWasFlipped = shouldFlip;
    } catch (Exception exception) {
      DriverStation.reportWarning(
          "Unable to preview selected auto path '" + selectedAuto + "': " + exception.getMessage(),
          false);
    }
  }

  private void updateAnimation() {
    if (previewPoses.isEmpty()) {
      return;
    }

    double totalDistance = getPreviewDistance();
    if (totalDistance <= 0.0) {
      field.setRobotPose(previewPoses.get(0));
      return;
    }

    double elapsedTime = Timer.getFPGATimestamp() - animationStartTime;
    double previewDistance = (elapsedTime * ANIMATION_SPEED_METERS_PER_SEC) % totalDistance;
    field.setRobotPose(samplePreviewPose(previewDistance));
  }

  private double getPreviewDistance() {
    double distance = 0.0;
    for (int i = 1; i < previewPoses.size(); i++) {
      distance +=
          previewPoses
              .get(i - 1)
              .getTranslation()
              .getDistance(previewPoses.get(i).getTranslation());
    }
    return distance;
  }

  private Pose2d samplePreviewPose(double previewDistance) {
    double traversedDistance = 0.0;

    for (int i = 1; i < previewPoses.size(); i++) {
      Pose2d startPose = previewPoses.get(i - 1);
      Pose2d endPose = previewPoses.get(i);
      double segmentDistance = startPose.getTranslation().getDistance(endPose.getTranslation());

      if (segmentDistance <= 0.0) {
        continue;
      }

      if (traversedDistance + segmentDistance >= previewDistance) {
        double interpolation = (previewDistance - traversedDistance) / segmentDistance;
        return new Pose2d(
            startPose.getTranslation().interpolate(endPose.getTranslation(), interpolation),
            endPose.getTranslation().minus(startPose.getTranslation()).getAngle());
      }

      traversedDistance += segmentDistance;
    }

    return previewPoses.get(previewPoses.size() - 1);
  }
}
