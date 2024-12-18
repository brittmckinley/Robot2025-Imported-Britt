// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desciption
 * This command takes in the chooser options and run the sequence of commands
 */
package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Camera.CameraPositionSetter;
import frc.robot.commands.ModifiedWPILibCommands.SequentialCommandGroupDynamic;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.MathExtendedUtil;
import java.util.ArrayList;

// Run the commands in parallel
public class GeneralAuto extends ParallelCommandGroup {

  private SequentialCommandGroupDynamic dynamicCommandGroup;

  public GeneralAuto(
    SwerveSubsystem m_swerveSubsystem,
    Camera m_camera,
    Intake m_intake,
    ArrayList<SendableChooser<Command>> steps, // Steps to execute
    SendableChooser<Pose2d> position // Where the robot is starting
  ) {
    addCommands(
      // Reset the odometry from where the robot is positioned.
      new InstantCommand(() -> {
        m_swerveSubsystem.resetOdometry(
          MathExtendedUtil.getAlliedFieldPosition(position.getSelected())
        );
      }),
      // creates commands when this command is run, not created (allows for us to
      // chose right before match)
      new SequentialCommandGroupDynamic(
        () -> {
          ArrayList<Command> commands = new ArrayList<>();
          // gets the commands selected on shuffleboard and sets them to run here
          for (SendableChooser<Command> commandChooser : steps) {
            if (commandChooser.getSelected() != null) {
              commands.add(commandChooser.getSelected());
            }
          }
          return commands;
        },
        m_swerveSubsystem,
        m_intake
      ),
      // maintains robot position using limelight
      new CameraPositionSetter(m_camera, m_swerveSubsystem)
    );
  }
}
