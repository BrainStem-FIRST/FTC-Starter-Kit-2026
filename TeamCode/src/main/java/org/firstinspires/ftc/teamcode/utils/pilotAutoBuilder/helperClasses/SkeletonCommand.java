package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import java.util.List;

@JsonIgnoreProperties(ignoreUnknown = true)
public class SkeletonCommand {
    public String id;
    public String type;
    public String label;
    public double defaultWait;
    public String subsystemName;
    public String commandName;
    public List<SkeletonCommand> parallelSubs;
}
