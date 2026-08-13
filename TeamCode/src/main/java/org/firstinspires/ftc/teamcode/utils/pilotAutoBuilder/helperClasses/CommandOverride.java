package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

@JsonIgnoreProperties(ignoreUnknown = true)
public class CommandOverride {
    public String cmdId;
    public boolean skip;
    public String pathId;
}
