//TODO BSM - Add file header comment block,  Add reference to where we got the code from.
// ^ I got no clue to be honest
package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.DriveConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class SwerveSubsystem extends SubsystemBase {

  // Create 4 Serve Modules using Port Numbers
  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort,
    DriveConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
  );

  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort,
    DriveConstants.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
    DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
  );

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort,
    DriveConstants.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
    DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
  );

  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort,
    DriveConstants.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
    DriveConstants.kBackRightDriveAbsoluteEncoderReversed
  );

  // Create NAVX Gyroscope using AHRS Library
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // Create Swerve Drive Odometry
  private final SwerveDrivePoseEstimator odometer;

  // Simulation Field for Shuffleboard
  private Field2d sim_field = new Field2d();

  private double gyroFieldOrientationOffset = 0; //offsets gyro to field orientation (because the robot won't start field oriented, but the gyro recalibration forces it to 0)
  //usually set by the april tags
  private double expectedYaw = 0; //what we expect the robot to currently be facing (used to counteract angle drift)

  private ChassisSpeeds previousChassisSpeeds = null;

  public SwerveSubsystem() {
    // Recalibrates gyro 1 second after robot intalizes to make sure gyro is online
    new Thread(() -> {
      try {
        gyro.setAngleAdjustment(0);
        Thread.sleep(1000);
        zeroHeading();
        resetOdometry(
          new Pose2d(
            Constants.FieldConstants.fieldSizeMeters.div(2),
            new Rotation2d(0)
          )
        ); //puts it default in center of field
      } catch (Exception e) {}
    })
      .start();

    SmartDashboard.putData("Field", sim_field);

    // Odometer is used to estimate where the robot is on the field with code
    // We initiate it here to assume it is at the origin (corner of the map)
    // We do add the current wheel configurations though so we don't need to align them after each match!
    //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
    //https://www.chiefdelphi.com/t/example-code-for-the-swervedriveposeestimator-class/423897
    odometer =
      new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        new Rotation2d(0),
        new SwerveModulePosition[] {
          frontLeft.getSwerveModulePosition(),
          frontRight.getSwerveModulePosition(),
          backLeft.getSwerveModulePosition(),
          backRight.getSwerveModulePosition(),
        },
        new Pose2d(0, 0, new Rotation2d(0))
      );
  }

  @Override
  public void periodic() {
    // Update Odometer Repeatedly to Accumulate location
    odometer.update(
      getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getSwerveModulePosition(),
        frontRight.getSwerveModulePosition(),
        backLeft.getSwerveModulePosition(),
        backRight.getSwerveModulePosition(),
      }
    );

    Pose2d currentEstimate = getPose();
    double[] poseArray = {
      currentEstimate.getX(),
      currentEstimate.getY(),
      currentEstimate.getRotation().getDegrees(),
    };

    //Critical info, used by depth camera thus outside ifs
    SmartDashboard.putNumberArray("Position", poseArray);
    SmartDashboard.putNumber("Swerve Drive Expected angle", expectedYaw);

    //resets inescapable NAN issue (odometer can't added to NAN)
    //start up issue -> makes sure nothing is invalid
    if (
      Double.isNaN(currentEstimate.getX()) ||
      Double.isNaN(currentEstimate.getY())
    ) {
      resetOdometry(new Pose2d(0, 0, getRotation2d()));
    }

    // Debug info to monitor the robot heading value
    if (DebugConstants.enableSmartDash) {
      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putString(
        "Robot Location",
        getPose().getTranslation().toString()
      );
    }

    addChild("FL-1", frontLeft); // debugging
    addChild("FR-2", frontRight);
    addChild("BL-3", backLeft);
    addChild("BR-4", backRight);

    sim_field.setRobotPose(getPose()); //shows on the sim field current robot position and rotation
  }

  /**
   * Resets the gyro to face 0
   */
  public void zeroHeading() {
    gyro.reset();
    expectedYaw = 0;
  }

  /**
   * Sets expected angle to the current heading
   * Means the robot should move, avoids big spins when going inbetween commands
   */
  public void setExpectedAngleToHeading() {
    expectedYaw = getHeading();
  }

  /**
   * Gets the gyro heading relative to field (0 is facing directly away from blue alliance wall)
   * In other terms, if we were on the the blue driver station, facing directly forward onto the field would be 0 to the robot
   * Goes CCW (CW = Clockwise, CCW = Counter Clock Wise)
   * @return heading clamped to -180 to 180.
   */
  public double getHeading() {
    return Math.IEEEremainder(
      -gyro.getAngle() + gyroFieldOrientationOffset, //offsets the gyro using the field (allows for field orientaions)
      //-gyro as it works CW cuz its goofy (gyro is CW while WPILIB is CCW)
      360
    );
  }

  /**
   * Gets robot heading encapsulated in Rotation2d
   * @see getHeading()
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * Gets estimated field position using the odometer
   * @return
   */
  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }

  /**
   * Sets the expected yaw (rotation on the floor) of the robot, used for telop drive
   * Used to counteract angle drift
   * @see RunSwerveTelop
   */
  public double getExpectedYaw() {
    return expectedYaw;
  }

  /**
   * Sets where the robot is expected to be
   *
   * @param expectedAngle
   */
  public void setExpectedYaw(double expectedAngle) {
    this.expectedYaw = expectedAngle;
  }

  // Reset Odometry
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(
      getRotation2d(),
      new SwerveModulePosition[] {
        frontLeft.getSwerveModulePosition(),
        frontRight.getSwerveModulePosition(),
        backLeft.getSwerveModulePosition(),
        backRight.getSwerveModulePosition(),
      },
      pose
    );
  }

  /**
   * Adds a measurement from limelight to the odometer
   * @param pose
   * @param latency
   */
  public void addVisionMeasurement(Pose2d pose, double latency) {
    odometer.addVisionMeasurement(
      pose,
      Timer.getFPGATimestamp() - latency,
      VecBuilder.fill(0.05, 0.05, Math.toRadians(30))
    );
  }

  /**
   * Creates a basic trajectory between two points with no intermediate position
   * @param intialLoc
   * @param finalLoc
   * @return
   */
  public Trajectory generateTrajectory(Pose2d intialLoc, Pose2d finalLoc) {
    return generateTrajectoryWithDiscreteIntermediates(
      intialLoc,
      finalLoc,
      Constants.DriveConstants.kTrajectoryConfig,
      List.of()
    );
  }

  /**
   * Generates a trajectory roughly
   * Draws straight lines between the each position (initial to intermediates to final)
   * instead of having smooth paths
   *
   * @param intialLoc
   * @param finalLoc
   * @param config
   * @param IntermediatePoses
   * @return
   */
  public Trajectory generateTrajectoryWithDiscreteIntermediates(
    Pose2d intialLoc,
    Pose2d finalLoc,
    TrajectoryConfig config,
    List<Pose2d> IntermediatePoses
  ) {
    ArrayList<Pose2d> waypointArrayList = new ArrayList<>(); //all poses in the trajectory
    if (IntermediatePoses != null) {
      waypointArrayList = new ArrayList<>(IntermediatePoses);
    }

    //adds final and intial locations into the array
    waypointArrayList.add(finalLoc);
    waypointArrayList.add(0, intialLoc); // At to beginning

    //current trajectory
    Trajectory currentTraj = null;
    //goes through each pair of waypoints and draws a line
    for (int i = 0; i < waypointArrayList.size() - 1; i++) {
      Pose2d firstPos = waypointArrayList.get(i);
      Pose2d secondPos = waypointArrayList.get(i + 1);
      TrajectoryConfig tempConfig = config;
      //gets the angle so that the created trajectory will be a line when we run the generator
      Translation2d deltaTranslation = secondPos
        .getTranslation()
        .minus(firstPos.getTranslation()); //Get delta X and delta Y

      Rotation2d deltaRot = deltaTranslation.getAngle(); //get arctan of deltaY/deltaX

      //sets the end velocity of each point to the max to maximize speed
      if (i != waypointArrayList.size() - 2) {
        tempConfig = config.setEndVelocity(config.getMaxVelocity());
      } else if (waypointArrayList.size() != 2) {
        //sets last point to go slower if needed -> was iffy
        // config = Constants.DriveConstants.kTrajectoryConfigSlower;
      }

      //sets start velocity to max if it isn't the start
      if (i != 0) {
        tempConfig = config.setStartVelocity(config.getMaxVelocity());
      }

      //THIS WAS EXPERIMENTAL
      //BASICALLY ADJUSTING SPEEDS BASED OFF OF HOW MANY INTERMEDIATE POSES WE HAD
      // INTENDED TO HELP SLOW THE ROBOT DOWN TO PICK UP NOTES BUT BE FAST ENOUGH IN BETWEEN NOTES
      //sorry ugly code but needs to be done now
      //USED FOR 2024 -> 2 intermediate positions were used ONLY FOR FAR NOTES -> NOT GENERALIZED AT ALL
      if (IntermediatePoses != null && IntermediatePoses.size() == 2) {
        if (i == 0) {
          tempConfig = Constants.DriveConstants.kTrajectoryConfig;
          tempConfig =
            config.setEndVelocity(
              Constants.DriveConstants.kTrajectoryConfigSlower.getMaxAcceleration()
            );
        } else if (i == 1) {
          tempConfig = config;
          tempConfig = tempConfig.setStartVelocity(config.getMaxVelocity());
        }
      }

      //generates the trajectory
      Trajectory newTraj = TrajectoryGenerator.generateTrajectory(
        new Pose2d(firstPos.getTranslation(), deltaRot), //replace??
        List.of(),
        new Pose2d(secondPos.getTranslation(), deltaRot),
        tempConfig
      );

      //adds the trajectory to the current list
      if (currentTraj == null) {
        currentTraj = newTraj;
      } else {
        currentTraj = currentTraj.concatenate(newTraj);
      }
    }

    return currentTraj;
  }

  /**
   * Adds the trajectory to the field for visualization (important for E/A Stops)
   * @param trajectory
   */
  public void setField(Trajectory trajectory) {
    sim_field.getObject("traj").setTrajectory(trajectory);
  }

  /**
   * Used for depth camera, generates trajectory given path of points given from ASTAR
   * @param intialPos
   * @param endPos
   * @param pointsXY points from depth camera [x1,y1,x2,y2,...]
   * @param previousVelocity
   * @return Robot trajectory
   */
  public Trajectory generateTrajectoryUsingMidPoints(
    Pose2d intialPos,
    Pose2d endPos,
    double[] pointsXY,
    double previousVelocity
  ) {
    ArrayList<Translation2d> points = new ArrayList<>();
    for (int i = 16; i < pointsXY.length - 1; i += 2 * 15) {
      points.add(new Translation2d(pointsXY[i], pointsXY[i + 1]));
    }

    TrajectoryConfig tc = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
    )
      .setKinematics(DriveConstants.kDriveKinematics)
      .setStartVelocity(previousVelocity);
    System.out.println(previousVelocity);

    //
    if (points.size() > 0) {
      Rotation2d angleToGo = endPos
        .getTranslation()
        .minus(intialPos.getTranslation())
        .getAngle();
      intialPos = new Pose2d(intialPos.getTranslation(), angleToGo);
    }
    Trajectory newTraj = null;
    try {
      newTraj =
        TrajectoryGenerator.generateTrajectory(intialPos, points, endPos, tc);
    } catch (Exception e) {
      System.out.println(e);
    }
    System.out.println(newTraj);
    return newTraj;
  }

  /**
   * Stops swerve motors
   */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /**
   * Sets swerve motors
   * @param desiredStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Normalize wheel speeds, proportionally decrease them below speed limit
    //normalizing speed vector
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      DriveConstants.kPhysicalMaxSpeedMetersPerSecond
    );
    if (Constants.DriveConstants.kFrontLeftDriveEnabled) {
      frontLeft.setDesiredState(desiredStates[0]);
    }
    if (Constants.DriveConstants.kFrontRightDriveEnabled) {
      frontRight.setDesiredState(desiredStates[1]);
    }
    if (Constants.DriveConstants.kBackLeftDriveEnabled) {
      backLeft.setDesiredState(desiredStates[2]);
    }
    if (Constants.DriveConstants.kBackRightDriveEnabled) {
      backRight.setDesiredState(desiredStates[3]);
    }
  }

  // Returns velocity of yaw in degrees
  public double getAngularVelocity() {
    return gyro.getRate();
  }

  // Function to Reset Gyro
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Offsets the gyro, used to offset for field (as robot starts at 0, which may not correspond to field position)
   * @param expectedZero
   */
  public void setGyro(double expectedZero) {
    gyroFieldOrientationOffset = expectedZero + gyro.getAngle();
    Pose2d currentEstimatedPos = odometer.getEstimatedPosition();
    this.resetOdometry(
        new Pose2d(
          currentEstimatedPos.getX(),
          currentEstimatedPos.getY(),
          new Rotation2d(Math.toRadians(expectedZero))
        )
      );
    expectedYaw = expectedZero;
  }

  /***
   * Resets odometry and field debug widget to the current trajectory -> run right before using a trajectory
   * @param trajectory
   */
  public void prepareTrajectoryToRun(Trajectory trajectory) {
    // this.resetOdometry(trajectory.getInitialPose());
    this.setField(trajectory);
  }

  //WPILIB modified command
  //Look at {@code}ChassisSpeeds.discretize(null, dtSeconds){@code}
  public static ChassisSpeeds discretize(
    ChassisSpeeds continuousSpeeds,
    double dtSeconds
  ) {
    return discretize(
      continuousSpeeds.vxMetersPerSecond,
      continuousSpeeds.vyMetersPerSecond,
      continuousSpeeds.omegaRadiansPerSecond,
      dtSeconds
    );
  }

  /**
   * Converts the continious chassis speeds to discrete
   * Helps account for robot drift
   * Look at {@code}ChassisSpeeds.discretize(null, dtSeconds){@code}
   * @param vxMetersPerSecond
   * @param vyMetersPerSecond
   * @param omegaRadiansPerSecond
   * @param dtSeconds
   * @return
   */
  public static ChassisSpeeds discretize(
    double vxMetersPerSecond,
    double vyMetersPerSecond,
    double omegaRadiansPerSecond,
    double dtSeconds
  ) {
    var desiredDeltaPose = new Pose2d(
      vxMetersPerSecond * dtSeconds,
      vyMetersPerSecond * dtSeconds,
      new Rotation2d(4.1 * omegaRadiansPerSecond * dtSeconds) //4.1 only change -> Constants
    );
    var twist = new Pose2d().log(desiredDeltaPose);
    return new ChassisSpeeds(
      twist.dx / dtSeconds,
      twist.dy / dtSeconds,
      omegaRadiansPerSecond
    );
  }

  //on the spot function
  //returns front left wheel speed -> used to determine if robot was moving
  //IDK why I squared it to be honest?
  public double getWheelSpeed() {
    double wheelMovement =
      frontLeft.getState().speedMetersPerSecond *
      frontLeft.getState().speedMetersPerSecond; //all other wheels will have the same speed;
    return Math.abs(wheelMovement);
  }
}
