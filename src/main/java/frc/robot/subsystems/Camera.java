// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.NetworkResult;
import java.util.ArrayList;
import java.util.LinkedHashMap;

// TODO BSM - Can you break this into limelight and depth camera classes? having them mixed is confusing
// ^ to this point 1. we didn't use depth camera :( but otherwise this could be implemented (maybe have an abstract class/interface for cameras in general?)
// My one argument against this is that this would treat one sensor as one subsystem, which seems too small
// I see both of these implementations to be fine
public class Camera extends SubsystemBase {

  private ObjectMapper mapper = new ObjectMapper();

  //See limelight table info here VVVV
  //https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
  private NetworkTable limelightTable = NetworkTableInstance
    .getDefault()
    .getTable("limelight");

  //See github project
  private NetworkTable depthTable = NetworkTableInstance
    .getDefault()
    .getTable("depthcamera");

  private DoubleArraySubscriber depthTrajectoryData;

  public Camera() {
    depthTable.getEntry("passive").setBoolean(false);
    depthTrajectoryData =
      depthTable
        .getDoubleArrayTopic("Trajectory Data")
        .subscribe(new double[] {});
  }

  @Override
  public void periodic() {}

  /***
   * Tells depth camera to calculate path from current robot position (uses "Position" stored in shuffleboard)
   */
  public void getDepthCameraData(Pose2d endPose) {
    depthTable
      .getEntry("Trajectory End Point")
      .setDoubleArray(new double[] { endPose.getX(), endPose.getY() });
    depthTable.getEntry("Trajectory Request").setBoolean(true);
    depthTable.getEntry("Trajectory Request Fulfilled").setBoolean(false);
  }

  public void resetDepthCameraRequest() {
    depthTable.getEntry("Trajectory Request Fulfilled").setBoolean(false);
  }

  public boolean getDepthTrajectoryRequestFulfilled() {
    return depthTable
      .getEntry("Trajectory Request Fulfilled")
      .getBoolean(false);
  }

  public double[] getDepthTrajectoryRaw() {
    return depthTrajectoryData.get();
  }

  public String getAprilTagJsonString() {
    return limelightTable.getEntry("json").getString("");
  }

  /**
   * Gets the first april tag detected in the list
   * @return
   */
  public Pose2d getAprilTagLoc() {
    return getAprilTagLoc(-1);
  }

  /* gets exact location of first april tag detected */
  // TODO - BSM - This could use a lot more explaination and a link to the
  // spec that you are using to decode the data. (????)
  // This used com.fasterxml.jackson.databind.ObjectMapper to parse JSON
  /**
   * @Deprecated
   * No longer used in the code, delete this next year <p>
   * Gets the location of an april tag with a certain id,
   * id = -1 just gets the first april tag in the detection array </p>
   * @param id (id of the april tag)
   * @return Pose2d of the april tag RELATIVE to the robot (using the robot as the origin)
   */

  public Pose2d getAprilTagLoc(int id) {
    String jsonString = getAprilTagJsonString(); //gets the json string from shuffleboard/the limelight

    ArrayList<Number> results = new ArrayList<>(); //our 6 number output (x,y,z,rx,ry,rz)

    if (jsonString == "") return null;
    try {
      // System.out.println(jsonString); ->debug purposes
      var a = mapper.readValue(jsonString, NetworkResult.class); //not sure why var but oh well, json throws a lot of errors so being safe
      ArrayList fiducialList = ((ArrayList) (a.getResults().get("Fiducial"))); //gets list of fiducials detected by the camera

      int index = 0;
      if (id != -1) { //filters by id if given an id, if no id given return the first one found
        for (int i = 0; i < fiducialList.size(); i++) {
          if (
            (Integer) ((LinkedHashMap) fiducialList.get(i)).get("fID") == (id)
          ) {
            index = i;
            break;
          }
        }
      }
      // System.out.println(fiducialList.size()); -> for debug purposes

      if (fiducialList.size() != 0) results =
        (ArrayList<Number>) (
          ((LinkedHashMap) fiducialList.get(index)).get("t6t_rs") //target in robot space -> where is the target relative to the robot (1 m forward, etc)
        );
    } catch (JsonProcessingException e1) {
      e1.printStackTrace();
    }

    if (results.size() < 1) return null;
    Double rx = ((Number) (results.get(3))).doubleValue();
    Double rz = ((Number) (results.get(5))).doubleValue();
    Double ry = ((Number) (results.get(4))).doubleValue();
    double angle = ry * Math.PI / 180;

    System.out.println(rx + " " + rz + " ROTATION " + angle);

    Double z = ((Number) (results.get(0))).doubleValue();
    Double x = ((Number) (results.get(2))).doubleValue();

    Pose2d result = new Pose2d(x, z, Rotation2d.fromRadians(angle));
    System.out.println(result);

    return result;
  }

  /**
   * Gets the robot position relative to the blue origin from shuffleboard
   * @return [x, y, z, roll, pitch, yaw, latency]
   */
  public double[] getRobotFieldPositionRaw() {
    return parseLimelightJson("botpose_wpiblue");
  }

  /**
   * Gets the robot position using the limelight
   * Uses {@code} getRobotFieldPositionRaw() {@code}
   * @return Pose2d of estimated robot position based soley from limelight
   */
  public Pose2d getRobotsFieldPosition() {
    double[] rawFieldLocation = getRobotFieldPositionRaw(); //get raw robot position on field
    SmartDashboard.putNumberArray("estimated field pos", rawFieldLocation);
    return new Pose2d(
      rawFieldLocation[0],
      rawFieldLocation[1],
      new Rotation2d(Math.toRadians(rawFieldLocation[5]))
    );
  }

  /***
   * Gets the latency of the camera (the difference in time of the data)
   * There is a very small lag in the data as the limelight needs to take a picture, then run all calculations to identify
   * an april tag (usually 40 ms between frame capture and data extraction). This function returns that lag
   * @return
   */
  public double getLatency() {
    return getRobotFieldPositionRaw()[6] / 1000;
  }

  /**
   * See function below in the source file as this is just an override
   * @param targetSpace
   * @return
   */
  private double[] parseLimelightJson(String targetSpace) {
    return parseLimelightJson(targetSpace, false);
  }

  /**
   * Gets a 7 double array which represents the robots position on the field (x,y,z,rx,ry,rz, latency)
   * See limelight docs for more info
   * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
   * @param targetSpace
   * @param latencyInfo
   * @return
   */
  private double[] parseLimelightJson(String targetSpace, boolean latencyInfo) {
    double results[] = limelightTable
      .getEntry(targetSpace)
      .getDoubleArray(new double[6 + ((latencyInfo) ? 1 : 0)]); //gets json -> will always be constants, latency adds one more value
    return results;
  }
}
