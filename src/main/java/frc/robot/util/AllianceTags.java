// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Restricts AprilTag tracking to the tags on our own alliance's half of the field.
 *
 * On the 2026 field IDs 1-16 live on the red half and IDs 17-32 on the blue half, so a
 * red robot only ever looks at 1-16 and a blue robot only ever looks at 17-32.
 *
 * The filter is pushed down to the Limelights themselves with
 * {@link LimelightHelpers#SetFiducialIDFiltersOverride(String, int[])}, so tv/tx/ta,
 * the fiducial ID and the MegaTag botposes all ignore the other alliance's tags.
 * {@link #isAllianceTag(double)} is the same check in Java for code that reads raw
 * fiducials, in case a camera reboots before the filter is re-applied.
 */
public class AllianceTags {

  /** AprilTag IDs on the red half of the field. */
  public static final int[] RED_TAG_IDS = {9,10,15,16,13,14};

  /** AprilTag IDs on the blue half of the field. */
  public static final int[] BLUE_TAG_IDS = {25,26,31,32,29,30};

  /** Every tag on the field, used when the filter is turned off. */
  public static final int[] ALL_TAG_IDS = IntStream.rangeClosed(1, 32).toArray();

  /** Set false (dashboard key "Vision/AllianceTagFilter") to let the cameras see every tag. */
  public static boolean enabled = true;

  // What we last pushed to each camera, so we only write when something changes.
  private static final Map<String, String> lastApplied = new HashMap<>();

  private AllianceTags() {}

  /** Returns true if we are on red. Prefers the Driver Station, falls back to the dashboard flag. */


  /** The tag IDs we are allowed to look at right now. */
  public static int[] getAllianceTagIDs() {
    if (!enabled) {
      return ALL_TAG_IDS;
    }
    return Robot.isRed() ? RED_TAG_IDS : BLUE_TAG_IDS;
  }

  /** True if the given tag is on our alliance's half of the field. */
  public static boolean isAllianceTag(int id) {
    if (!enabled) {
      return true;
    }
    if (id < 1 || id > 32) {
      return false;
    }
    return Robot.isRed() ? id <= 16 : id >= 17;
  }

  /** True if the given tag is on our alliance's half of the field. -1 (no tag) is never ours. */
  public static boolean isAllianceTag(double id) {
    return isAllianceTag((int) id);
  }

  /**
   * True if a pose estimate was built only from our own alliance's tags. Estimates with no
   * tags, or with any opponent-side tag mixed in, are rejected.
   */
  public static boolean isAllianceEstimate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null || estimate.pose == null || estimate.tagCount <= 0) {
      return false;
    }
    if (!enabled) {
      return true;
    }
    if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      // No per-tag detail available; the camera-side filter is the only guard we have.
      return true;
    }
    for (var fiducial : estimate.rawFiducials) {
      if (!isAllianceTag(fiducial.id)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Pushes the alliance tag filter down to the given Limelights. Only writes when the alliance
   * (or the enable toggle) changes, so this is safe to call every loop.
   */
  public static void update(String... limelightNames) {
    enabled = SmartDashboard.getBoolean("Vision/AllianceTagFilter", enabled);
    SmartDashboard.putBoolean("Vision/AllianceTagFilter", enabled);

    boolean red = Robot.isRed();
    String state = enabled ? (red ? "Red 1-16" : "Blue 17-32") : "All 1-32";
    SmartDashboard.putString("Vision/VisibleTags", state);

    int[] validIDs = getAllianceTagIDs();
    for (String name : limelightNames) {
      // Each camera is tracked separately so a subsystem that only owns one camera
      // can't stop the others from being filtered.
      if (state.equals(lastApplied.get(name))) {
        continue;
      }
      LimelightHelpers.SetFiducialIDFiltersOverride(name, validIDs);
      lastApplied.put(name, state);
    }
  }

  /** Forces the next {@link #update(String...)} to re-push the filter (e.g. after a camera reboot). */
  public static void forceReapply() {
    lastApplied.clear();
  }
}
