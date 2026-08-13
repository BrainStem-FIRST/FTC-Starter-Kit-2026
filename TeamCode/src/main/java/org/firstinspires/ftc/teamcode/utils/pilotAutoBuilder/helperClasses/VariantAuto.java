package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import java.util.List;

@JsonIgnoreProperties(ignoreUnknown = true)
public class VariantAuto {
    public String name;
    public String skeletonId;
    public List<CommandOverride> commandOverrides;
}
