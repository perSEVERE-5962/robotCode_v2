package frc.robot.util;

import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

/**
 * Utility wrapper for Elastic Dashboard notifications.
 * Provides simple static methods for sending notifications at different severity levels.
 */
public final class ElasticUtil {

    private ElasticUtil() {
        // Prevent instantiation
    }

    /**
     * Sends an informational notification to Elastic Dashboard.
     * @param title The notification title
     * @param message The notification message
     */
    public static void sendInfo(String title, String message) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.INFO, title, message)
        );
    }

    /**
     * Sends a warning notification to Elastic Dashboard.
     * @param title The notification title
     * @param message The notification message
     */
    public static void sendWarning(String title, String message) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.WARNING, title, message)
        );
    }

    /**
     * Sends an error notification to Elastic Dashboard.
     * @param title The notification title
     * @param message The notification message
     */
    public static void sendError(String title, String message) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.ERROR, title, message)
        );
    }

    /**
     * Sends an informational notification with custom display time.
     * @param title The notification title
     * @param message The notification message
     * @param displayTimeMs How long to display the notification in milliseconds
     */
    public static void sendInfo(String title, String message, int displayTimeMs) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.INFO, title, message, displayTimeMs)
        );
    }

    /**
     * Sends a warning notification with custom display time.
     * @param title The notification title
     * @param message The notification message
     * @param displayTimeMs How long to display the notification in milliseconds
     */
    public static void sendWarning(String title, String message, int displayTimeMs) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.WARNING, title, message, displayTimeMs)
        );
    }

    /**
     * Sends an error notification with custom display time.
     * @param title The notification title
     * @param message The notification message
     * @param displayTimeMs How long to display the notification in milliseconds
     */
    public static void sendError(String title, String message, int displayTimeMs) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.ERROR, title, message, displayTimeMs)
        );
    }
}
