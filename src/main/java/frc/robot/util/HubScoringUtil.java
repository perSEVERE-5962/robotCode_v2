// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class HubScoringUtil {

  public static Pose2d getClosestScoringPose(Translation2d robotPosition, Translation2d hubCenter, 
                                            double scoringDistance, Rotation2d scoringSide, double arcWidthDegrees) {
    
    Translation2d hubToRobot = robotPosition.minus(hubCenter); // calculates line from center of hub to the robot
    
   
    double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX());//calculates the angle of that line on a circle
    
    
    double centerAngle = scoringSide.getRadians();//gets the angle that the arc is around
    double halfArcRadians = Math.toRadians(arcWidthDegrees / 2.0);//splits the angle in half, both +-1/2totalangle around the center of the angle.
    
    
    double angleDiff = angleToRobot - centerAngle;//calculate the difference between desired angle vs current angle
    angleDiff = Math.atan2(Math.sin(angleDiff), Math.cos(angleDiff));//WRAPS THE ANGLE USING MATH(this took a while)
    
    
    double clampedAngleDiff = Math.max(-halfArcRadians, Math.min(halfArcRadians, angleDiff));//the current angle on the arc of the robot relative to the hub is then clamped to the scoring arc(determines closest angle on arc)
    double targetAngle = centerAngle + clampedAngleDiff;//sets the target angle on the arc as nearest point on the arc
    
    //caclulate the pose on the arc
    Translation2d targetPosition = new Translation2d(
        hubCenter.getX() + scoringDistance * Math.cos(targetAngle),
        hubCenter.getY() + scoringDistance * Math.sin(targetAngle)
    );
    
    // Robot should face toward the hub(opposite of direction on the angle is center of hub)
    Rotation2d targetRotation = Rotation2d.fromRadians(targetAngle + Math.PI);
    
    return new Pose2d(targetPosition, targetRotation);
  }


}
