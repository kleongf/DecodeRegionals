package org.firstinspires.ftc.teamcode.lib.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class CommandScheduler {
    private static final CommandScheduler instance = new CommandScheduler();

    public static CommandScheduler getInstance() {
        return instance;
    }

    private final List<Subsystem> subsystems = new ArrayList<>();
    private final List<Command> activeCommands = new ArrayList<>();
    private final Map<Subsystem, Command> requirements = new HashMap<>();

    private CommandScheduler() {}

    void registerSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void schedule(Command command) {
        if (activeCommands.contains(command)) return;

        Set<Command> conflicting = new HashSet<>();
        for (Subsystem subsystem : command.getRequirements()) {
            Command owner = requirements.get(subsystem);
            if (owner != null) conflicting.add(owner);
        }
        for (Command owner : conflicting) cancel(owner);

        command.start();
        activeCommands.add(command);
        for (Subsystem subsystem : command.getRequirements()) {
            requirements.put(subsystem, command);
        }
    }

    public void cancel(Command command) {
        if (!activeCommands.remove(command)) return;
        command.interrupt();
        requirements.values().removeIf(owner -> owner == command);
    }

    public void cancelAll() {
        for (Command command : new ArrayList<>(activeCommands)) cancel(command);
    }

    public boolean isScheduled(Command command) {
        return activeCommands.contains(command);
    }

    public boolean isBusy(Subsystem subsystem) {
        return requirements.containsKey(subsystem);
    }

    public void run() {
        for (Subsystem subsystem : subsystems) subsystem.periodic();
        for (Command command : new ArrayList<>(activeCommands)) {
            command.periodic();
            if (command.isFinished()) {
                activeCommands.remove(command);
                requirements.values().removeIf(owner -> owner == command);
            }
        }
    }

    void reset() {
        subsystems.clear();
        activeCommands.clear();
        requirements.clear();
    }
}
