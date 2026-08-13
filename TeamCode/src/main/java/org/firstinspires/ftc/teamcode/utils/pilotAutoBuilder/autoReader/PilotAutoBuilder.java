package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.autoReader;

import com.acmerobotics.roadrunner.Action;

/** Fluent builder for BrainstemPilot autos and standalone paths. */
public class PilotAutoBuilder {

    enum Target {
        AUTO,
        PATH
    }

    private final String m_name;
    private final Target m_target;

    PilotAutoBuilder(String name, Target target) {
        m_name = name;
        m_target = target;
    }

    static PilotAutoBuilder forAuto(String variantAutoName) {
        return new PilotAutoBuilder(variantAutoName, Target.AUTO);
    }

    static PilotAutoBuilder forPath(String pathId) {
        return new PilotAutoBuilder(pathId, Target.PATH);
    }

    public Action build() {
        return m_target == Target.AUTO
                ? BrainstemPilot.buildAutoInternal(m_name)
                : BrainstemPilot.buildPathInternal(m_name);
    }
}
