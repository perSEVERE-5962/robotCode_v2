package frc.robot.util;

import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

/** Wrapper for Elastic Dashboard notifications. */
public final class ElasticUtil {

    private ElasticUtil() {}

    public static void sendInfo(String title, String message) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.INFO, title, message)
        );
    }

    public static void sendWarning(String title, String message) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.WARNING, title, message)
        );
    }

    public static void sendError(String title, String message) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.ERROR, title, message)
        );
    }

    public static void sendInfo(String title, String message, int displayTimeMs) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.INFO, title, message, displayTimeMs)
        );
    }

    public static void sendWarning(String title, String message, int displayTimeMs) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.WARNING, title, message, displayTimeMs)
        );
    }

    public static void sendError(String title, String message, int displayTimeMs) {
        Elastic.sendNotification(
            new Notification(NotificationLevel.ERROR, title, message, displayTimeMs)
        );
    }
}
