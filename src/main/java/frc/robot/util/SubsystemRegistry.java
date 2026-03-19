package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

public final class SubsystemRegistry {
    private final Map<Class<? extends Subsystem>, Subsystem> subsystems = new LinkedHashMap<>();

    public <T extends Subsystem> T register(T subsystem) {
        @SuppressWarnings("unchecked")
        Class<T> subsystemType = (Class<T>) subsystem.getClass();
        return register(subsystemType, subsystem);
    }

    public <T extends Subsystem> T register(Class<T> type, T subsystem) {
        Objects.requireNonNull(type, "type");
        Objects.requireNonNull(subsystem, "subsystem");

        Subsystem existingSubsystem = subsystems.putIfAbsent(type, subsystem);
        if (existingSubsystem != null && existingSubsystem != subsystem) {
            throw new IllegalStateException("Subsystem already registered for type " + type.getName());
        }

        return subsystem;
    }

    public <T extends Subsystem> Optional<T> get(Class<T> type) {
        Objects.requireNonNull(type, "type");
        return Optional.ofNullable(type.cast(subsystems.get(type)));
    }

    public <T extends Subsystem> T require(Class<T> type) {
        return get(type)
                .orElseThrow(() -> new IllegalStateException("No subsystem registered for type " + type.getName()));
    }

    public boolean contains(Class<? extends Subsystem> type) {
        Objects.requireNonNull(type, "type");
        return subsystems.containsKey(type);
    }

    public <T> List<T> all(Class<T> type) {
        Objects.requireNonNull(type, "type");

        List<T> matches = new ArrayList<>();
        Set<T> seen = Collections.newSetFromMap(new IdentityHashMap<>());
        for (Subsystem subsystem : subsystems.values()) {
            if (!type.isInstance(subsystem)) {
                continue;
            }

            T match = type.cast(subsystem);
            if (seen.add(match)) {
                matches.add(match);
            }
        }

        return List.copyOf(matches);
    }

    public List<Subsystem> all() {
        return all(Subsystem.class);
    }
}
