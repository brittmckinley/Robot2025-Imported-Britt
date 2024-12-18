package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.AccelerationConstraint;

public final class Constants {

  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kDriveEncoderRot2Meter =
      kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad =
      kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec =
      kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec =
      kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  public static final class DriveConstants {

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(21.5);

    public static final double kRobotWidth = Units.inchesToMeters(30);

    public static final double kDrivingHeadingTolerance = 5;

    // Set the swerve kinematics based on the chassis measurements. -> negatives are important
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Can Bus Ids for each module
    public static final int kFrontLeftDriveMotorPort = 12;
    public static final int kBackLeftDriveMotorPort = 22;
    public static final int kFrontRightDriveMotorPort = 42;
    public static final int kBackRightDriveMotorPort = 32;

    public static final int kFrontLeftTurningMotorPort = 11;
    public static final int kBackLeftTurningMotorPort = 21;
    public static final int kFrontRightTurningMotorPort = 41;
    public static final int kBackRightTurningMotorPort = 31;

    // Direction of rotation for the moter encoders per corner
    // To set turn the wheels in the same direction and note if the encoder
    // values are going up or down.
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    // Direction of the wheel encoders
    // To set turn the wheels in the same direction and note if the
    // drive encoders are going up or down.
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    // Can Bus IDs for the cancoders of each corner
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 10;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 20;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 40;
    public static final int kBackRightDriveAbsoluteEncoderPort = 30;

    // Te set these turn the wheels in the same direction relative to the
    // robot and see if the absolute encoder are going up or down.
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    // Offset of the absolute encoders in degrees.
    // To set these values align all the wheels to forward and read the value of
    // the encoders.
    public static final double kFrontLeftDriveAbsoluteEncoderOffset = -36.562; // +180; // ID 10
    public static final double kBackLeftDriveAbsoluteEncoderOffset = -161.279; // +180;// -341.1+180; // ID 20
    public static final double kFrontRightDriveAbsoluteEncoderOffset = -173.848; // +180;// -170.508; // ID 40 !!!
    public static final double kBackRightDriveAbsoluteEncoderOffset = -275.011; // +180;// -273.516; // ID 30

    // Sets the maximum speeds for the chassis
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5; //4.75;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond =
      2 * 2 * Math.PI;

    //
    public static final double kTeleDriveMaxSpeedMetersPerSecond =
      kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
      kPhysicalMaxAngularSpeedRadiansPerSecond / 1.5;
    // Limit to drive acceleration in meters per second.
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    // Limit the turning acceleration in meters per second
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond =
      8;
    public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2
    )
      .setKinematics(DriveConstants.kDriveKinematics)
      .addConstraint(
        new AccelerationConstraint(
          -Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared / 3,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared / 2
        )
      );

    public static final TrajectoryConfig kTrajectoryConfigSlower = new TrajectoryConfig(
      0.45 * Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      2 * Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    )
      .addConstraint(
        new AccelerationConstraint(
          -0.4 * Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
      );

    public static final TrajectoryConfig kTrajectoryConfigSuperSlow = new TrajectoryConfig(
      0.3 * Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared * 1.5
    )
      .setKinematics(DriveConstants.kDriveKinematics)
      .addConstraint(
        new AccelerationConstraint(
          -Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared * 1.5
        )
      );
    // FOR DEBUGING: Enable/Disable certain motors
    public static boolean kFrontLeftDriveEnabled = true;
    public static boolean kBackLeftDriveEnabled = true;
    public static boolean kFrontRightDriveEnabled = true;
    public static boolean kBackRightDriveEnabled = true;
  }

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond =
      0.8 * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    public static final double kMaxAngularSpeedRadiansPerSecond =
      DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.4;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared =
      Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularAccelerationRadiansPerSecondSquared
    );

    public static final int kNumberOfAutoSteps = 5;
  }

  public static final class IntakeConstants {

    public static final int kFlipMotorID = 51;
    public static final int kWheelMotorID = 52;

    public static final int kAbsoluteEncoderID = 4;

    public static final int kUpperLimitSwitchChannel = 3;
    public static final int kLowerLimitSwitchChannel = 2;

    //feedforward value for arm (could be used to make motion smoother)
    public static final double kFlipMotorKV = 0.04; //0.04;

    //Used to detect whether we have a note inside
    public static final double kWheelOutputCurrentLimit = 25; //35 ma, above this we have a note
    public static final double kIntakeWheelCurrentTime = 0.2;

    //times
    public static final double kHandShakeTime = 0.05;
    public static final double kFlipPIDTimeout = 0.5;

    //Speeds
    public static double kIntakeSpeedIntake = 0.65;
    public static double kIntakeSpeedFeeding = -0.4;

    //angles deployed, etc.
    public static final double kAngleHandshake = 176;
    public static final double kAngleDeployed = 0;
  }

  public static final class ShooterConstants {

    public static final int kAimingMotorID = 60;

    public static final int kLeftShootMotorID = 62;
    public static final int kWRightShootMotorID = 61;

    public static final int kShooterAbsoluteEncoder = 5;

    public static final int kIRBeam = 7;
    public static final int kUpperLimitSwitch = 6;

    //Pid values (kv is feedforward value)
    public static double kShooterFlipPIDTimeout = 1;
    public static double kFlipPIDKv = 0.04;

    //angles
    public static double kShooterAngleAmp = 295;
    public static double kShooterAngleHandshake = 21;
    public static double kShooterAngleSpeaker = 21;
    public static double kShooterAngleParallel = 320;

    //speeds
    public static double kShooterSpeedSpeaker = 0.8;
    public static double kShooterSpeedDrop = 0.15;
    public static double kShooterSpeedAmp = 0.55;
    public static double kShooterSpeedFeed = 0.12;

    //time to spin up
    public static double kSpinUpTime = 0.35;
  }

  public static final class ClimberConstants {

    public static final int forwardChannel = 14;
    public static final int reverseChannel = 15;
    public static final int hubPort = 1;
  }

  public static final class CameraConstants {

    public static final double kDetectionTime = 0.2;
  }

  public static final class OIConstants {

    public static final int kDriverControllerPort = 0;

    //Axis
    public static final int kDriverYAxis = 1; // 1;
    public static final int kDriverXAxis = 0; // 0;
    public static final int kDriverRotAxis = 4; // 4;

    //Buttons
    public static final int kMoveShooterBtn = 1; //USED IN TELOP
    public static final int kAutoDriveBtn = 2;

    public static final int kToggleIntakeBtn = 3;
    public static final int kShootShooterBtn = 4;

    public static final int kDecreaseSensitivity = 5;
    public static final int kIncreaseSensitivity = 6;

    public static final double kDeadband = 0.1;

    public static final int kToggleClimberBtn = 7; // X button (xbox)
    public static final int kToggleIntakeWheelsBtn = 8; // X button (xbox)
  }

  public static final class DebugConstants {

    public static final boolean debugMode = true;
    public static final boolean enableSmartDash = true;
  }

  // Data obtained by using Pathweaver (moving the waypoints to a location on the image shown on pathweaver, then copying x,y, theta)
  // theta obtained just by eye balling/assuming where we would want the robot to be facing (this would be atan(y/x) of the rotaiton config )
  // This is manual work
  public static final class LocationConstants {

    public static final Pose2d blueAmpLocation = new Pose2d(
      1.85,
      7.65,
      new Rotation2d(Math.toRadians(-90))
    );
    public static final Pose2d blueIntakeLocation = new Pose2d(
      15.5,
      1,
      new Rotation2d(Math.toRadians(35))
    );

    public static final Pose2d blueUpperSideOfSpeaker = new Pose2d(
      1.5,
      6.5,
      new Rotation2d(Math.toRadians(180 - 45))
    );
    public static final Pose2d blueFrontSideOfSpeaker = new Pose2d(
      1.5,
      5.55,
      new Rotation2d(Math.toRadians(180))
    );
    public static final Pose2d blueLowerSideOfSpeaker = new Pose2d(
      1.25,
      4.6,
      new Rotation2d(Math.toRadians(225))
    );

    //close note locations
    public static final Pose2d blueUpperNoteLocation = new Pose2d(
      2.9,
      7,
      new Rotation2d(Math.toRadians(45))
    );
    public static final Pose2d blueMiddleNoteLocation = new Pose2d(
      2.9,
      5.55,
      new Rotation2d(Math.toRadians(0))
    );
    public static final Pose2d blueLowerNoteLocation = new Pose2d(
      2.9,
      4.0,
      new Rotation2d(Math.toRadians(-45))
    );

    //center line note locations
    public static final Pose2d middleUpperNoteLocation = new Pose2d(
      8.3,
      7.45,
      new Rotation2d(0)
    );
    public static final Pose2d middleCenterUpperNoteLocation = new Pose2d(
      8.3,
      5.75,
      new Rotation2d(0)
    );
    public static final Pose2d middleCenterMiddleNoteLocation = new Pose2d(
      8.3,
      4.1,
      new Rotation2d(0)
    );
    public static final Pose2d middleCenterLowerNoteLocation = new Pose2d(
      8.3,
      2.4,
      new Rotation2d(0)
    );
    public static final Pose2d middleLowerNoteLocation = new Pose2d(
      8.3,
      0.75,
      new Rotation2d(0)
    );

    public static final Pose2d blueLowerStageLocation = new Pose2d(
      4,
      1,
      new Rotation2d(0)
    );
    public static final Pose2d blueUpperStageLocation = new Pose2d(
      6,
      7,
      new Rotation2d(0)
    );

    public static final Pose2d centerOfStage = new Pose2d(
      1.35,
      5.5,
      new Rotation2d(Math.toRadians(180))
    );
    public static final Pose2d lowerStageSide = new Pose2d(
      0.504,
      4.195,
      new Rotation2d(Math.toRadians(180 - 60))
    ); //15.956
    public static final Pose2d lowerCenterStage = new Pose2d(
      0.712,
      4.316,
      new Rotation2d(Math.toRadians(180 - 60))
    ); //15.956
    public static final Pose2d UpperStageSide = new Pose2d(
      0.504,
      6.85,
      new Rotation2d(Math.toRadians(180 + 60))
    ); //15.956
    public static final Pose2d UpperCenterStage = new Pose2d(
      0.712,
      6.85 - 0.121,
      new Rotation2d(Math.toRadians(180 + 60))
    ); //15.956

    // public static final Pose2d upperStage = new Pose2d(15.25, 4.456, new Rotation2d(Math.PI));

    /**<p>
     * We use the blue cordinates for every match, even if we are on red, thus we need to transform our cordinates when on red
     * (those cordinates being used for auto pilot, i.e. the prebaked location data shown above)
     * Look at the image on pathweaver for more details, but cordinates need to essentially be mirrored over the center line </p>
     * <pre>
     * Example 1:
     *
     * | * -> * * CENTER LINE * * * * |           | * * * * CENTER LINE * * <- * |
     * | *  * * * CENTER LINE * * * * |   - TO -  | * * * * CENTER LINE * * *  * |
     * | *  * * * CENTER LINE * * * * |           | * * * * CENTER LINE * * *  * |
     *
     * Example 2: (shortening y axis because that wont change)
     * | *  ↑ * * CENTER LINE * * * * |   - TO -  | * * * * CENTER LINE * *  ↑ * |
     * </pre>
     * <p>
     * As you see, the robot is reflected along the y-axis of the field, thus the y field doesn't change,
     * But the x axis is going to be FIELD LENGTH - CURRENT POSITION (if the robot is on the wall, then
     * on the blue side this would be x=0, but on the red team this would be x=FIELD LENGTH
     * At the center line, on the blue side x = FIELD LENGTH/2, and for the red team this is also X=FIELD LENGTH/2
     * Rotation is also mirrored (which is done by 180 - current Rotation)
     * </p>
     **/
    public static Pose2d convertBlueToRedPose(Pose2d position) {
      return new Pose2d(
        FieldConstants.fieldSizeMeters.getX() - position.getX(),
        position.getY(),
        position.getRotation().minus(new Rotation2d(Math.PI)).times(-1) //-1*(x-180) = 180-x
        //position.getRotation().times(-1).plus(new Rotation2d(Math.PI))
      );
    }

    //used to calculate shortest speaker distance
    public static final Pose2d blueSpeakerCenter = new Pose2d(
      0,
      5.5,
      new Rotation2d(0)
    ); //center of speaker(on the wall)
    public static final double blueSpeakerRadiusMeters = 1.3; //radius from the center of the speaker where the robot can make a shot
    public static final double kAlignedWithNoteDistanceMeters = 1.2; //distance before note where it is a straight line to it (distane where the robot intake should be directly looking at the note)
  }

  public static final class FieldConstants {

    public static final Translation2d fieldSizeMeters = new Translation2d(
      16.541748984,
      8.21055
    ); //from pathweaver files (could be confirmed with game manual), size of the field in meters
    //used to swap between blue and red positiosn (see convertBlueToRedPOse)

  }
}
