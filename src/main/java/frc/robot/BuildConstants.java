package frc.robot;

/**
 * Build constants for AdvantageKit metadata logging.
 *
 * These values are recorded at startup and visible in AdvantageScope's Metadata tab.
 *
 * TODO: Consider using gversion Gradle plugin to auto-generate this file with
 * actual git info. See: https://docs.advantagekit.org/getting-started/installation/version-control/
 */
public final class BuildConstants {
    /** Maven project name */
    public static final String MAVEN_NAME = "robotcode2026-lab";

    /** Build timestamp (update manually or use gversion plugin) */
    public static final String BUILD_DATE = "2026-01-19";

    /** Git commit SHA (update manually or use gversion plugin) */
    public static final String GIT_SHA = "unknown";

    /** Git commit date */
    public static final String GIT_DATE = "unknown";

    /** Git branch name */
    public static final String GIT_BRANCH = "feature/logging-tamara";

    /** 0 = clean, 1 = dirty (uncommitted changes), -1 = unknown */
    public static final int DIRTY = -1;

    private BuildConstants() {
        // Prevent instantiation
    }
}
