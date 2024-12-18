package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.util.Optional;

/**
 * A class used to work with field positions
 * Extends the WPILib class, {@code} MathUtil {@code}
 * @see edu.wpi.first.math.util
 */
public class MathExtendedUtil {

  /**
   * Checks if a limelight position is accutally valid by seeing if it is within the field
   * @param poseInQuestion
   * @return if the position is on the match field
   */
  public static boolean isPoseOnField(Pose2d poseInQuestion) {
    boolean result = true;
    if (poseInQuestion.getX() < 0 || poseInQuestion.getY() < 0) {
      result = false;
    }
    if (
      poseInQuestion.getX() > Constants.FieldConstants.fieldSizeMeters.getX() ||
      poseInQuestion.getY() > Constants.FieldConstants.fieldSizeMeters.getY()
    ) {
      result = false;
    }
    return result;
  }

  /**
   * Converts the blue positions to actual locations based off of our alliance color
   * @param blueFieldPosition - location where we want to go ON BLUE FIELD
   * @return location where we want to go BASED ON OUR ALLIANCE COLOR
   */
  public static Pose2d getAlliedFieldPosition(Pose2d blueFieldPosition) {
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent()) {
      if (currentAlliance.get() == Alliance.Blue) {
        return blueFieldPosition;
      } else {
        return Constants.LocationConstants.convertBlueToRedPose(
          blueFieldPosition
        );
      }
    }
    return blueFieldPosition; //rip but yea
  }
}
