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
    public static final String MAVEN_NAME = "robotcode2026-lab";
    public static final String BUILD_DATE = "2026-01-19"; // TODO(Tamara): use gversion plugin
    public static final String GIT_SHA = "unknown";
    public static final String GIT_DATE = "unknown";
    public static final String GIT_BRANCH = "feature/logging-tamara";
    public static final int DIRTY = -1; // 0=clean, 1=dirty, -1=unknown

    private BuildConstants() {}
}
