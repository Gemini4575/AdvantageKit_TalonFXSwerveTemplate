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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Displays a selected PathPlanner auto and animates its robot along the preview path. */
public class AutoPreview {
  private static final double DEFAULT_ANIMATION_SPEED_METERS_PER_SEC = 2.0;

  private final Field2d field;
  private final Supplier<String> selectedAutoSupplier;
  private final BooleanSupplier shouldFlipSupplier;
  private final double animationSpeedMetersPerSec;

  private String lastPreviewedAuto = "";
  private boolean lastPreviewWasFlipped;
  private List<Pose2d> previewPoses = List.of();
  private double animationStartTime;

  /**
   * Creates a preview that follows PathPlanner's current alliance flip setting.
   *
   * @param field field widget to update and animate
   * @param selectedAutoSupplier supplier for the selected PathPlanner auto name; use "None" for no
   *     selection
   */
  public AutoPreview(Field2d field, Supplier<String> selectedAutoSupplier) {
    this(field, selectedAutoSupplier, AutoBuilder::shouldFlip);
  }

  /**
   * Creates a preview with an explicit flip setting source.
   *
   * @param field field widget to update and animate
   * @param selectedAutoSupplier supplier for the selected PathPlanner auto name; use "None" for no
   *     selection
   * @param shouldFlipSupplier supplier for whether paths should be flipped
   */
  public AutoPreview(
      Field2d field, Supplier<String> selectedAutoSupplier, BooleanSupplier shouldFlipSupplier) {
    this(field, selectedAutoSupplier, shouldFlipSupplier, DEFAULT_ANIMATION_SPEED_METERS_PER_SEC);
  }

  /**
   * Creates a preview with explicit inputs and animation speed.
   *
   * @param field field widget to update and animate
   * @param selectedAutoSupplier supplier for the selected PathPlanner auto name; use "None" for no
   *     selection
   * @param shouldFlipSupplier supplier for whether paths should be flipped
   * @param animationSpeedMetersPerSec animation speed in meters per second
   */
  public AutoPreview(
      Field2d field,
      Supplier<String> selectedAutoSupplier,
      BooleanSupplier shouldFlipSupplier,
      double animationSpeedMetersPerSec) {
    this.field = field;
    this.selectedAutoSupplier = selectedAutoSupplier;
    this.shouldFlipSupplier = shouldFlipSupplier;
    this.animationSpeedMetersPerSec = animationSpeedMetersPerSec;
  }

  /** Refreshes the path display when the selected auto or flip setting changes. */
  public void update() {
    String selectedAuto = selectedAutoSupplier.get();
    boolean shouldFlip = shouldFlipSupplier.getAsBoolean();

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

  /** Advances the robot marker along the currently displayed path. */
  public void updateAnimation() {
    if (previewPoses.isEmpty()) {
      return;
    }

    double totalDistance = getPreviewDistance();
    if (totalDistance <= 0.0) {
      field.setRobotPose(previewPoses.get(0));
      return;
    }

    double elapsedTime = Timer.getFPGATimestamp() - animationStartTime;
    double previewDistance = (elapsedTime * animationSpeedMetersPerSec) % totalDistance;
    field.setRobotPose(samplePreviewPose(previewDistance));
  }

  /** Returns the field widget owned by this preview. */
  public Field2d getField() {
    return field;
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
