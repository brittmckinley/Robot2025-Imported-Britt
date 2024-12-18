// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ModifiedWPILibCommands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * A command composition that runs a list of commands in sequence.
 * Intalizes all the commands when this is initalized, NOT when the command is created
 *
 */
public class SequentialCommandGroupDynamic extends Command {

  private Supplier<ArrayList<Command>> commands;
  private final List<Command> m_commands = new ArrayList<>();
  private int m_currentCommandIndex = -1;
  private boolean m_runWhenDisabled = true;
  private InterruptionBehavior m_interruptBehavior =
    InterruptionBehavior.kCancelIncoming;

  /**
   * Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the
   * composition finishing when the last command finishes.
   *
   * @param commands the commands to include in this composition.
   */
  public SequentialCommandGroupDynamic(
    Supplier<ArrayList<Command>> commands,
    Subsystem... requirements
  ) {
    this.commands = commands;
    addRequirements(requirements);
  }

  /**
   * Adds the given commands to the group.
   *
   * @param commands Commands to add, in order of execution.
   */
  public final void addCommands(Command... commands) {
    if (m_currentCommandIndex != -1) {
      throw new IllegalStateException(
        "Commands cannot be added to a composition while it's running"
      );
    }

    //MODIFED AREA
    // CommandScheduler.getInstance().clearComposedCommands();

    for (Command command : commands) {
      CommandScheduler.getInstance().clearComposedCommands();
    }
    //END OF MODIFIED AREA

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (Command command : commands) {
      m_commands.add(command);
      // m_requirements.addAll(command.getRequirements());
      m_runWhenDisabled &= command.runsWhenDisabled();
      if (
        command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf
      ) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    // CommandScheduler.getInstance().
    //MODIFIED AREA
    m_commands.clear();
    // m_requirements.clear();
    m_runWhenDisabled = true;
    m_currentCommandIndex = -1;
    m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

    // CommandScheduler.getInstance().

    Command[] commandArray = this.commands.get().toArray(new Command[0]); //covnerts arraylist to array
    System.out.println(commands.get());
    System.out.println(commands.get().size());
    System.out.println(commandArray[0].getName());
    System.out.println(commandArray[0].getRequirements());

    addCommands(commandArray);
    m_currentCommandIndex = 0;
    //END OF MODIFIED AREA

    if (!m_commands.isEmpty()) {
      m_commands.get(0).initialize();
    }
  }

  @Override
  public final void execute() {
    if (m_commands.isEmpty()) {
      return;
    }

    Command currentCommand = m_commands.get(m_currentCommandIndex);

    currentCommand.execute();
    if (currentCommand.isFinished()) {
      currentCommand.end(false);
      m_currentCommandIndex++;
      if (m_currentCommandIndex < m_commands.size()) {
        m_commands.get(m_currentCommandIndex).initialize();
      }
    }
  }

  @Override
  public final void end(boolean interrupted) {
    if (
      interrupted &&
      !m_commands.isEmpty() &&
      m_currentCommandIndex > -1 &&
      m_currentCommandIndex < m_commands.size()
    ) {
      m_commands.get(m_currentCommandIndex).end(true);
    }
    m_currentCommandIndex = 0;
  }

  @Override
  public final boolean isFinished() {
    return m_currentCommandIndex == m_commands.size();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addIntegerProperty("index", () -> m_currentCommandIndex, null);
  }
}
