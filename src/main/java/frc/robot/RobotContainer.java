package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands.GeneralAuto;
import frc.robot.commands.AutoCommands.ModuleCommands.JustBackUp;
import frc.robot.commands.AutoCommands.ModuleCommands.Nada;
import frc.robot.commands.Camera.CameraPositionSetter;
import frc.robot.commands.DriveTrain.AutoDrive;
import frc.robot.commands.DriveTrain.DEBUG.CreateTrajectoryUsingDepth;
import frc.robot.commands.DriveTrain.DEBUG.DriveToTagGroup;
import frc.robot.commands.DriveTrain.DriveToPositionUsingDepth;
import frc.robot.commands.DriveTrain.RunSwerveTeleop;
import frc.robot.commands.Lights.RunLightsTeleop;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;

public class RobotContainer {

  // Create the Subsystems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Camera cameraSubsystem = new Camera();
  // private final Intake m_intake = new Intake();
  // private final Shooter m_shooter = new Shooter();
  // private final Lights m_lights = new Lights();
  // private final Climber climberSubsystem = new Climber();

  // Set up the autonomous chooser
  private final ArrayList<SendableChooser<Command>> autoChoosers = new ArrayList<>();
  private final SendableChooser<Pose2d> positionOfRobot = new SendableChooser<>();

  // New Joystick command
  private final Joystick driverJoytick = new Joystick(
    OIConstants.kDriverControllerPort
  );

  // Make joystick command default for the swerve chassis!
  public RobotContainer() {
    //Setting default commands (run continiously unless we need a command that uses (sets) multiple subsystems)
    //some commands may take in some subsystems, but not require them (ONLY do this for getters, nothing should be set here)
    // Each subsystem has a matching default command that it will run when another command is not running.
    swerveSubsystem.setDefaultCommand(
      new RunSwerveTeleop(
        swerveSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
        // opposite of joystick button to be in field relative mode by default
        () -> driverJoytick.getPOV(),
        () -> false,
        () -> false,
        () -> driverJoytick.getRawButton(OIConstants.kToggleIntakeWheelsBtn)
      )
    );

    cameraSubsystem.setDefaultCommand(
      new CameraPositionSetter(cameraSubsystem, swerveSubsystem)
    );

    // m_intake.setDefaultCommand(
    //   new RunIntakeTeleop(
    //     m_intake,
    //     m_shooter,
    //     () -> driverJoytick.getRawButton(OIConstants.kToggleIntakeBtn),
    //     () -> false
    //   )
    // );

    // m_shooter.setDefaultCommand(
    //   new RunShooterTeleop(
    //     m_shooter,
    //     () -> driverJoytick.getRawButton(OIConstants.kMoveShooterBtn)
    //   )
    // );

    // m_lights.setDefaultCommand(
    //   new RunLightsTeleop(m_lights, m_shooter, m_intake, swerveSubsystem)
    // );

    //SETTING DEBUGS

    // Smart Dashboard Key to Drive to Center of April Tag
    SmartDashboard.putData(
      "AutoDrive-Center",
      new DriveToTagGroup(
        swerveSubsystem,
        cameraSubsystem,
        () -> 0.0,
        () -> 0.0
      )
    );
    // Add Buttons on the smart dashboard that will run a command when it is pressed.
    // These are for debugging
    // SmartDashboard.putData("Open Climber", new OpenClimber(climberSubsystem));

    // SmartDashboard.putData("Close Climber", new CloseClimber(climberSubsystem));

    SmartDashboard.putData(
      "Send Data Request to Depth Camera",
      new InstantCommand(() -> {
        cameraSubsystem.getDepthCameraData(new Pose2d(1, 1, new Rotation2d(0)));
      })
    );

    // SmartDashboard.putData(
    //   "Toggle Intake",
    //   new ConditionalCommand(
    //     new MoveIntake(m_intake, Constants.IntakeConstants.kAngleDeployed),
    //     new MoveIntake(m_intake, Constants.IntakeConstants.kAngleHandshake),
    //     () ->
    //       (
    //         m_intake.getCurrentAngle() -
    //         Constants.IntakeConstants.kAngleHandshake /
    //         2 >
    //         0 &&
    //         m_intake.getCurrentAngle() <
    //         Constants.IntakeConstants.kAngleHandshake +
    //         20
    //       )
    //   )
    // );

    // SmartDashboard.putData("Toggle Shooter", new ActivateShooter(m_shooter));

    SmartDashboard.putData(
      "Generate Traj with Depth",
      new CreateTrajectoryUsingDepth(
        cameraSubsystem,
        swerveSubsystem,
        () -> new Pose2d(10, 6, new Rotation2d(Math.PI / 2))
      )
    );

    SmartDashboard.putData(
      "Drive to pos with depth traj",
      new DriveToPositionUsingDepth(
        swerveSubsystem,
        cameraSubsystem,
        () -> Constants.LocationConstants.blueAmpLocation
      )
    );

    // SmartDashboard.putData(
    //   "Move Shooter to AMP",
    //   new MoveShooter(m_shooter, Constants.ShooterConstants.kShooterAngleAmp)
    // );
    // SmartDashboard.putData(
    //   "Move Shooter to Speaker",
    //   new MoveShooter(
    //     m_shooter,
    //     Constants.ShooterConstants.kShooterAngleSpeaker
    //   )
    // );
    // SmartDashboard.putData(
    //   "Move Shooter to Handshake",
    //   new MoveShooter(
    //     m_shooter,
    //     Constants.ShooterConstants.kShooterAngleHandshake
    //   )
    // );

    SmartDashboard.putNumber("Swerve Rotation Override", 0);
    SmartDashboard.putData(
      "Override Swerve Angle",
      new InstantCommand(() ->
        swerveSubsystem.setExpectedYaw(
          SmartDashboard.getNumber("Swerve Rotation Override", 0)
        )
      )
    );

    SmartDashboard.putData(
      "Swerve Reset Heading (set field)",
      new InstantCommand(() -> swerveSubsystem.zeroHeading())
    );

    //configure joystick buttons
    configureButtonBindings();

    //configure auto selectors
    // This sets the sequence of things to do then in auto
    for (int i = 0; i < Constants.AutoConstants.kNumberOfAutoSteps; i++) {
      SendableChooser<Command> newChooser = new SendableChooser<>();
      addAutoSelectorOptions(newChooser);
      autoChoosers.add(newChooser);
      SmartDashboard.putData("Autonomous Step " + i, newChooser);
    }
    // Set starting point
    createPositionOfRobotSelector(positionOfRobot);

    SmartDashboard.putData("Position setter", positionOfRobot);
  }

  // Set up the autonomous selector for where the robot is starting.  It will return
  // a constant where to stop.
  private void createPositionOfRobotSelector(SendableChooser<Pose2d> selector) {
    selector.addOption(
      "Center Of Speaker",
      Constants.LocationConstants.centerOfStage
    );
    selector.addOption(
      "Lower Of Speaker Near Wall",
      Constants.LocationConstants.lowerStageSide
    );
    selector.addOption(
      "Upper Of Speaker Near Wall",
      Constants.LocationConstants.UpperStageSide
    );
    selector.addOption(
      "Upper Of Speaker Centered",
      Constants.LocationConstants.UpperCenterStage
    );
    selector.setDefaultOption(
      "Lower Of Speaker Centered",
      Constants.LocationConstants.lowerCenterStage
    );
  }

  // Selector for what actions to take during auto.  It takes in the step selector.
  private void addAutoSelectorOptions(SendableChooser<Command> selector) {
    selector.setDefaultOption("Nada", new Nada());

    //Preload shot
    // selector.addOption(
    //   "Shoot into Speaker",
    //   new SequentialCommandGroup(
    //     new ShootSpeaker(m_shooter, m_intake, cameraSubsystem, swerveSubsystem),
    //     new InstantCommand(() ->
    //       m_shooter.setShooterMotors(
    //         Constants.ShooterConstants.kShooterSpeedSpeaker
    //       )
    //     ),
    //     new WaitCommand(0.7)
    //   )
    // );

    //Back outs to a position on amp side
    // selector.addOption(
    //   "Back out AMP side",
    //   new JustBackUp(
    //     swerveSubsystem,
    //     m_intake,
    //     m_shooter,
    //     cameraSubsystem,
    //     new Pose2d(2.75, 7.9, new Rotation2d(0))
    //   )
    // );

    // Back out to position on the intake side
    // selector.addOption(
    //   "Back out INTAKE side",
    //   new JustBackUp(
    //     swerveSubsystem,
    //     m_intake,
    //     m_shooter,
    //     cameraSubsystem,
    //     new Pose2d(2.75, 0.4, new Rotation2d(0))
    //   )
    // );

    //note scoring (close)
    // Go get the lower note then return ans score it.
    // selector.addOption(
    //   "Score Close Lower Note",
    //   new GetExtraNoteCloseAndScore(
    //     swerveSubsystem,
    //     m_shooter,
    //     m_intake,
    //     cameraSubsystem,
    //     Constants.LocationConstants.blueLowerNoteLocation
    //   )
    // );

    // Get the middle note and score it.
    // selector.addOption(
    //   "Score Close Middle Note",
    //   new SequentialCommandGroup(
    //     new ParallelDeadlineGroup(
    //       new WaitCommand(0.4),
    //       new MoveIntake(m_intake, Constants.IntakeConstants.kAngleDeployed)
    //     ),
    //     new GetExtraNoteCloseAndScore(
    //       swerveSubsystem,
    //       m_shooter,
    //       m_intake,
    //       cameraSubsystem,
    //       Constants.LocationConstants.blueMiddleNoteLocation
    //     )
    //   )
    // );
    // Get upper note and score it.
    // selector.addOption(
    //   "Score Close Upper Note",
    //   new GetExtraNoteCloseAndScore(
    //     swerveSubsystem,
    //     m_shooter,
    //     m_intake,
    //     cameraSubsystem,
    //     Constants.LocationConstants.blueUpperNoteLocation
    //   )
    // );

    //note scoring (center line)
    // selector.addOption(
    //   "Score Far Upper Note",
    //   new GetExtraNoteFarAndScore(
    //     swerveSubsystem,
    //     m_intake,
    //     m_shooter,
    //     Constants.LocationConstants.middleUpperNoteLocation
    //   )
    // );
    // selector.addOption(
    //   "Score Far Center Upper Note",
    //   new GetExtraNoteFarAndScore(
    //     swerveSubsystem,
    //     m_intake,
    //     m_shooter,
    //     Constants.LocationConstants.middleCenterUpperNoteLocation
    //   )
    // );
    // selector.addOption(
    //   "Score Far Center Middle Note",
    //   new GetExtraNoteFarAndScore(
    //     swerveSubsystem,
    //     m_intake,
    //     m_shooter,
    //     Constants.LocationConstants.middleCenterMiddleNoteLocation
    //   )
    // );
    // selector.addOption(
    //   "Score Far Center Lower Note",
    //   new GetExtraNoteFarAndScore(
    //     swerveSubsystem,
    //     m_intake,
    //     m_shooter,
    //     Constants.LocationConstants.middleCenterLowerNoteLocation
    //   )
    // );
    // selector.addOption(
    //   "Score Far Lower Note",
    //   new GetExtraNoteFarAndScore(
    //     swerveSubsystem,
    //     m_intake,
    //     m_shooter,
    //     Constants.LocationConstants.middleLowerNoteLocation
    //   )
    // );
  }

  //INCASE WE NEED TO SWITCH BACK
  // private void configureAutoSelector() {
  //   m_autoChooser.setDefaultOption("Nothing", new Nada());
  //   m_autoChooser.addOption(
  //     "Just Back up",
  //     new JustBackUp(
  //       swerveSubsystem,
  //       m_intake,
  //       m_shooter,
  //       cameraSubsystem,
  //       new Pose2d(2.5, 0.5, new Rotation2d(0))
  //     )
  //   );

  //   m_autoChooser.addOption(
  //     "Shoot and Back up",
  //     new ShootAndBackUp(
  //       swerveSubsystem,
  //       m_intake,
  //       m_shooter,
  //       cameraSubsystem,
  //       new Pose2d(2.5, 0.5, new Rotation2d(0))
  //     )
  //   );
  //   m_autoChooser.addOption(
  //     "Shoot and score 1 extra Lower",
  //     new ShootSpeakerAndScoreExtra(
  //       swerveSubsystem,
  //       m_shooter,
  //       m_intake,
  //       cameraSubsystem,
  //       Constants.LocationConstants.blueLowerNoteLocation
  //     )
  //   );
  //   m_autoChooser.addOption(
  //     "Shoot and score 1 extra Middle",
  //     new ShootSpeakerAndScoreExtra(
  //       swerveSubsystem,
  //       m_shooter,
  //       m_intake,
  //       cameraSubsystem,
  //       Constants.LocationConstants.blueMiddleNoteLocation
  //     )
  //   );
  //   m_autoChooser.addOption(
  //     "Shoot and score 1 extra Upper",
  //     new ShootSpeakerAndScoreExtra(
  //       swerveSubsystem,
  //       m_shooter,
  //       m_intake,
  //       cameraSubsystem,
  //       Constants.LocationConstants.blueUpperNoteLocation
  //     )
  //   );

  //   m_autoChooser.addOption(
  //     "Shoot and score 2 extra Lower + Middle",
  //     new ShootSpeakerAndScoreTwoExtra(
  //       swerveSubsystem,
  //       m_shooter,
  //       m_intake,
  //       cameraSubsystem,
  //       Constants.LocationConstants.blueMiddleNoteLocation,
  //       Constants.LocationConstants.blueLowerNoteLocation
  //     )
  //   );

  //   m_autoChooser.addOption(
  //     "Shoot and score 3 extra all close",
  //     new ShootSpeakerAndScoreThreeExtraClose(
  //       swerveSubsystem,
  //       m_shooter,
  //       m_intake,
  //       cameraSubsystem,
  //       Constants.LocationConstants.blueUpperNoteLocation,
  //       Constants.LocationConstants.blueMiddleNoteLocation,
  //       Constants.LocationConstants.blueLowerNoteLocation

  //     )
  //   );
  //   m_autoChooser.addOption(
  //     "Shoot and score 1 far",
  //     new ShootSpeakerAndScoreExtraFar(
  //       swerveSubsystem,
  //       m_shooter,
  //       m_intake,
  //       cameraSubsystem,
  //       Constants.LocationConstants.middleUpperNoteLocation
  //     )
  //   );

  //   m_autoChooser.addOption(
  //     "Just Shoot",
  //     new ShootSpeaker(m_shooter, m_intake, cameraSubsystem, swerveSubsystem)
  //   );

  //   SmartDashboard.putData(m_autoChooser);
  // }

  private void configureButtonBindings() {
    // JoystickButton moveShootButton = new JoystickButton(driverJoytick, OIConstants.kMoveShooterBtn);
    // moveShootButton.onTrue(new ) 1 USED in telop

    JoystickButton activateAutodrive = new JoystickButton(
      driverJoytick,
      OIConstants.kAutoDriveBtn
    );
    activateAutodrive.onTrue(
      new AutoDrive(
        swerveSubsystem,
        cameraSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis)
      )
    );

    // Sensitivity button used to get the note out of the intake
    JoystickButton kickNoteIntake = new JoystickButton(
      driverJoytick,
      OIConstants.kIncreaseSensitivity
    );
    // kickNoteIntake.onTrue(new KickOutNote(m_intake));
    // 3 USED in telop
    // JoystickButton shootShooter = new JoystickButton(
    //   driverJoytick,
    //   OIConstants.kShootShooterBtn
    // );
    // shootShooter.toggleOnTrue(new ShootWithSaftey(m_shooter, m_intake));


    // JoystickButton toggleClimber = new JoystickButton(
    //   driverJoytick,
    // JoystickButton shootShooter = new JoystickButton(
    //   driverJoytick,
    //   OIConstants.kShootShooterBtn
    // );

    // shootShooter.toggleOnTrue(new ShootWithSaftey(m_shooter, m_intake));

    // JoystickButton toggleClimber = new JoystickButton(
    //   driverJoytick,
    //   OIConstants.kToggleClimberBtn
    // );
    // toggleClimber.onTrue(
    //   new InstantCommand(() -> climberSubsystem.toggleClimber())
    // );
  }

  //gets the general auto command
  // public Command getAutonomousCommand() {
  //   return new GeneralAuto(
  //     swerveSubsystem,
  //     cameraSubsystem,
  //     m_shooter,
  //     m_intake,
  //     autoChoosers,
  //     positionOfRobot
  //   );
  // }
  public Command getAutonomousCommand() {
    return new GeneralAuto(
      swerveSubsystem,
      cameraSubsystem,
      null, autoChoosers,
      positionOfRobot
    );
  }
}
