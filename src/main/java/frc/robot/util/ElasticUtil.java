package frc.robot.util;

import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

/** Defensive wrapper for Elastic Dashboard notifications. */
public final class ElasticUtil {

    private ElasticUtil() {}

    public static void sendInfo(String title, String message) {
        try {
            Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, title, message)
            );
        } catch (Throwable t) {
        }
    }

    public static void sendWarning(String title, String message) {
        try {
            Elastic.sendNotification(
                new Notification(NotificationLevel.WARNING, title, message)
            );
        } catch (Throwable t) {
        }
    }

    public static void sendError(String title, String message) {
        try {
            Elastic.sendNotification(
                new Notification(NotificationLevel.ERROR, title, message)
            );
        } catch (Throwable t) {
        }
    }

    public static void sendInfo(String title, String message, int displayTimeMs) {
        try {
            Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, title, message, displayTimeMs)
            );
        } catch (Throwable t) {
        }
    }

    public static void sendWarning(String title, String message, int displayTimeMs) {
        try {
            Elastic.sendNotification(
                new Notification(NotificationLevel.WARNING, title, message, displayTimeMs)
            );
        } catch (Throwable t) {
        }
    }

    public static void sendError(String title, String message, int displayTimeMs) {
        try {
            Elastic.sendNotification(
                new Notification(NotificationLevel.ERROR, title, message, displayTimeMs)
            );
        } catch (Throwable t) {
        }
    }
}
