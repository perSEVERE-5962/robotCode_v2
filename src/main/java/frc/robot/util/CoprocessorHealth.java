package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Rolls multiple cameras that share a coprocessor into a single alive boolean, a reboot counter,
 * and a last-reboot timestamp. Solves the WPI Day 1 symptom where both rear cameras disconnected
 * together every few minutes and the dashboard showed two independent camera flickers instead of
 * the real root cause (an Orange Pi power cycle).
 *
 * <p>The detector treats a group as rebooting when every camera in the group goes offline within a
 * small time window. The recovery window is longer, which captures the real Orange Pi boot time of
 * about 20 seconds. A single camera dropping on its own does not count as a coprocessor reboot.
 *
 * <p>Thread-safety: not thread-safe. All updates happen on the scheduler thread via
 * VisionTelemetry.
 */
public final class CoprocessorHealth {

  /** Cameras that drop within this window are considered a paired loss. */
  private static final double PAIRED_DROP_WINDOW_SEC = 0.5;

  /** After this much time alive again, the group is considered recovered from a reboot. */
  private static final double RECOVERY_DWELL_SEC = 2.0;

  private static final class GroupState {
    final List<String> cameraNames = new ArrayList<>();
    boolean alive = false;
    boolean wasAlive = false;
    double firstDropTimestamp = -1;
    boolean rebootPending = false;
    int rebootCount = 0;
    double lastRebootTimestamp = 0;
    double aliveSinceTimestamp = -1;
  }

  private final Map<String, GroupState> groups = new HashMap<>();
  private final DoubleSupplier clock;

  /** Production constructor uses the WPILib FPGA timestamp as the clock. */
  public CoprocessorHealth() {
    this(() -> Timer.getFPGATimestamp());
  }

  /** Test constructor: inject a clock supplier to make reboot detection deterministic. */
  public CoprocessorHealth(DoubleSupplier clock) {
    this.clock = clock;
  }

  /**
   * Registers a camera as a member of a coprocessor group. Call once at construction time for each
   * expected camera. Unknown groups are created on first registration.
   */
  public void register(String cameraName, String coprocessorGroup) {
    GroupState group = groups.computeIfAbsent(coprocessorGroup, k -> new GroupState());
    if (!group.cameraNames.contains(cameraName)) {
      group.cameraNames.add(cameraName);
    }
  }

  /**
   * Drives the detector with the current connectivity state of each camera. Called once per cycle
   * from VisionTelemetry.update.
   *
   * @param cameraConnected a map of camera name to current connected boolean. Missing entries are
   *     treated as disconnected.
   */
  public void update(Map<String, Boolean> cameraConnected) {
    double now = clock.getAsDouble();
    for (GroupState group : groups.values()) {
      updateGroup(group, cameraConnected, now);
    }
  }

  private void updateGroup(GroupState group, Map<String, Boolean> cameraConnected, double now) {
    int aliveCount = 0;
    int totalCount = group.cameraNames.size();
    for (String cam : group.cameraNames) {
      Boolean state = cameraConnected.get(cam);
      if (state != null && state) {
        aliveCount++;
      }
    }
    group.wasAlive = group.alive;
    group.alive = aliveCount > 0;

    // Edge: alive -> dead, start the paired drop window.
    if (group.wasAlive && !group.alive) {
      group.firstDropTimestamp = now;
      group.aliveSinceTimestamp = -1;
    }

    // All cameras in the group dropped within the paired window: flag a pending reboot.
    // Single-camera drops (aliveCount == 0 but totalCount == 1) also count if the group has
    // only one camera, because a single-camera group IS the coprocessor.
    if (!group.alive
        && group.firstDropTimestamp >= 0
        && (now - group.firstDropTimestamp) <= PAIRED_DROP_WINDOW_SEC
        && aliveCount == 0) {
      group.rebootPending = true;
    }

    // Edge: dead -> alive, start the recovery dwell.
    if (!group.wasAlive && group.alive) {
      group.aliveSinceTimestamp = now;
    }

    // Recovery: group has been alive for long enough after a pending reboot.
    if (group.rebootPending
        && group.alive
        && group.aliveSinceTimestamp >= 0
        && (now - group.aliveSinceTimestamp) >= RECOVERY_DWELL_SEC) {
      group.rebootCount++;
      group.lastRebootTimestamp = now;
      group.rebootPending = false;
      group.firstDropTimestamp = -1;
    }
  }

  public boolean isGroupAlive(String groupName) {
    GroupState group = groups.get(groupName);
    return group != null && group.alive;
  }

  public int getRebootCount(String groupName) {
    GroupState group = groups.get(groupName);
    return group != null ? group.rebootCount : 0;
  }

  public double getLastRebootTimestamp(String groupName) {
    GroupState group = groups.get(groupName);
    return group != null ? group.lastRebootTimestamp : 0;
  }

  public double getTimeSinceLastRebootSec(String groupName) {
    GroupState group = groups.get(groupName);
    if (group == null || group.lastRebootTimestamp == 0) {
      return Double.POSITIVE_INFINITY;
    }
    return clock.getAsDouble() - group.lastRebootTimestamp;
  }

  /** Sum of reboot counts across all registered groups. */
  public int getTotalRebootCount() {
    int total = 0;
    for (GroupState group : groups.values()) {
      total += group.rebootCount;
    }
    return total;
  }

  /** True if any group has rebooted in the last {@code windowSec}. */
  public boolean anyRebootedRecently(double windowSec) {
    double now = clock.getAsDouble();
    for (GroupState group : groups.values()) {
      if (group.lastRebootTimestamp > 0 && (now - group.lastRebootTimestamp) <= windowSec) {
        return true;
      }
    }
    return false;
  }

  /** Returns the set of group names currently registered. */
  public Iterable<String> getGroupNames() {
    return groups.keySet();
  }
}
