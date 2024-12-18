// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ModuleCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveTrain.DriveToPositionAndRotate;
import frc.robot.commands.Intake.DEBUG.MoveIntake;
import frc.robot.commands.Intake.DEBUG.RunIntakeTilNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.MathExtendedUtil;
/* Descrition:
 * Runs a basic command that just shoots into the speaker.
 */

import java.util.List;
import java.util.function.Supplier;

public class GoToNoteAndPickUp extends SequentialCommandGroup {

  private List<Pose2d> prevPos; //intermediate positions (0.4 meters before )
  private Rotation2d goingBackRotation; // angle to head back to and rotate async
  private Pose2d fieldOrientedNoteLoc;

  /**
   * Moves robot to a location, picks up note, then drives back to another location
   * @param m_swerveSubsystem
   * @param m_intake
   * @param m_shooter
   * @param noteLocation    - position of the note to pick up
   * @param angleFacing     - angle to rotate to to pick up the note
   */
  public GoToNoteAndPickUp(
    SwerveSubsystem m_swerveSubsystem,
    Intake m_intake,
    Pose2d noteLocation,
    Supplier<Double> angleFacing
  ) {
    addCommands(
      // Step 1 - Get the current position on the field
      new InstantCommand(() -> {
        this.fieldOrientedNoteLoc =
          MathExtendedUtil.getAlliedFieldPosition(noteLocation);
        this.prevPos =
          List.of(
            new Pose2d(
              this.fieldOrientedNoteLoc.getX() +
              Constants.LocationConstants.kAlignedWithNoteDistanceMeters *
              Math.cos(Math.toRadians(angleFacing.get())),
              this.fieldOrientedNoteLoc.getY() +
              Constants.LocationConstants.kAlignedWithNoteDistanceMeters *
              Math.sin(Math.toRadians(angleFacing.get())),
              this.fieldOrientedNoteLoc.getRotation()
            )
          );
        this.fieldOrientedNoteLoc =
          new Pose2d(
            fieldOrientedNoteLoc.getX() +
            0.3 *
            Constants.DriveConstants.kRobotWidth /
            2 *
            Math.cos(Math.toRadians(angleFacing.get())),
            fieldOrientedNoteLoc.getY() +
            0.3 *
            Constants.DriveConstants.kRobotWidth /
            2 *
            Math.sin(Math.toRadians(angleFacing.get())),
            fieldOrientedNoteLoc.getRotation()
          );
      }),
      // Step 2 - - Try to get the note
      //   Run these commands until one finishes
      //   Either the note it intaked or a position is reached.
      new ParallelRaceGroup(
        new ParallelCommandGroup(
          new MoveIntake(m_intake, Constants.IntakeConstants.kAngleDeployed),
          new RunIntakeTilNote(m_intake)
          // new WaitCommand(0.2)
        ),
        new SequentialCommandGroup(
          // new RotateToAngle(m_swerveSubsystem, () -> angleFacing.get()),
          // new WaitCommand(1.5) -> may still be useful lol, needs testing
          new DriveToPositionAndRotate(
            m_swerveSubsystem,
            () -> fieldOrientedNoteLoc,
            DriveConstants.kTrajectoryConfigSlower,
            () -> prevPos,
            () -> new Rotation2d(Math.toRadians(angleFacing.get())),
            1
          )
        )
      ),
      // Step 3 - Drive back to the speaker?
      new ParallelCommandGroup(
        // new RunIntakeTilNote(m_intake, m_shooter),
        new SequentialCommandGroup(
          //
          new InstantCommand(() -> {
            Pose2d robotPos = m_swerveSubsystem.getPose();
            Pose2d speakerPos = MathExtendedUtil.getAlliedFieldPosition(
              Constants.LocationConstants.blueSpeakerCenter
            );
            Rotation2d angle = robotPos
              .getTranslation()
              .minus(speakerPos.getTranslation())
              .getAngle();
            goingBackRotation = angle.minus(new Rotation2d(Math.PI));
          }),
          // In parallel - drive back to the speaker and flip the intake
          new ParallelCommandGroup(
            new MoveIntake(m_intake, Constants.IntakeConstants.kAngleHandshake),
            new DriveToPositionAndRotate(
              m_swerveSubsystem,
              () -> calculateShortestPoseToSpeaker(m_swerveSubsystem),
              () -> {
                return this.goingBackRotation;
              },
              0
            )
          )
        )
      )
    );
  }

  //
  public Pose2d calculateShortestPoseToSpeaker(SwerveSubsystem m_swerve) {
    Pose2d currentPose = m_swerve.getPose();
    Pose2d speakerPosition = MathExtendedUtil.getAlliedFieldPosition(
      Constants.LocationConstants.blueSpeakerCenter
    );

    double deltaX = -speakerPosition.getX() + currentPose.getX();
    double deltaY = -speakerPosition.getY() + currentPose.getY();
    double deltaDistance = Math.pow((deltaX * deltaX + deltaY * deltaY), 0.5);
    deltaX *=
      Constants.LocationConstants.blueSpeakerRadiusMeters / deltaDistance;
    deltaY *=
      Constants.LocationConstants.blueSpeakerRadiusMeters / deltaDistance;

    return new Pose2d(
      speakerPosition.getX() + deltaX,
      speakerPosition.getY() + deltaY,
      new Rotation2d(Math.atan2(deltaY, deltaX) + Math.PI)
    );
  }
}
