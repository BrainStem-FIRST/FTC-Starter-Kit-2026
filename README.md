# FTC Starter Kit

An FTC robot controller project for the current season: SDK 11.2.1 plus Sloth
hot reload, FTC Dashboard, Panels, and BrainSTEM Pilot.

This is the stock FTC SDK (`FtcRobotController` + `TeamCode`) with those tools
added in `TeamCode` only. `FtcRobotController` is untouched so season SDK
updates can be merged from upstream. See
[Updating for a new season](#updating-for-a-new-season).

SDK docs: [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
· [ftc-docs](https://ftc-docs.firstinspires.org)

## Features

- **Sloth** — push TeamCode changes to the robot in under a second
- **Panels** and **FTC Dashboard** — live telemetry, tuning, and field view
- **BrainSTEM Pilot** — JSON-driven autonomous builder
- Mecanum drive with a Bézier path follower

## Requirements

- **JDK 17+** (Android Studio’s bundled JDK is fine)
- **Gradle 8.14.5** — pinned in `gradle/wrapper/gradle-wrapper.properties`. Do
  not bump to Gradle 9 until Sloth supports it

## Sloth

[Sloth](https://github.com/Dairy-Foundation/Sloth) hot-reloads classes under
`org.firstinspires.ftc.teamcode` instead of installing a full APK. Gradle
files, new dependencies, or anything outside that package still need a normal
full install.

**Setup (once)**

1. In the `TeamCode` run configuration, set **Module** to the option that is
   not `<no module>`.
2. Do one full `TeamCode` install.
3. After that, use **Sloth Load** (`.run/Sloth Load.run.xml` →
   `:TeamCode:deploySloth`).

Connect the Control Hub over USB-C, or Wi-Fi then
`adb connect 192.168.43.1:5555` (Control Hub) /
`adb connect 192.168.49.1:5555` (phone RC). Confirm with `adb devices`.

## Panels + FTC Dashboard

Both [Panels](https://panels.bylazar.com) and FTC Dashboard are wired in via
Sloth-compatible forks so they hot-reload with your code. Do not add the
vanilla artifacts alongside these — they clash at package level. Road Runner
is already declared with `exclude group: 'com.acmerobotics.dashboard'` for the
same reason.

- **Panels**: `http://192.168.43.1:8001` (Control Hub) or
  `http://192.168.49.1:8001` (Phone RC)
- **FTC Dashboard**: `http://192.168.43.1:8080/dash` (Control Hub) or
  `http://192.168.49.1:8080/dash` (Phone RC)

### Tunable values

Use both `@Configurable` (Panels) and `@Config` (Dashboard) — they are separate
systems:

```java
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
@Config
public class RobotConstants {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
}
```

### Telemetry

One `update()` reaches Panels, Dashboard, and the Driver Station:

```java
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;

TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
Telemetry dashboardAndDs = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

panelsTelemetry.addData("Example", "value");
panelsTelemetry.update(dashboardAndDs);
```

(`PanelsTelemetry` is a Kotlin `object`, so from Java use `.INSTANCE`.)

## BrainSTEM Pilot

BrainSTEM Pilot is a visual editor for autonomous routines. You draw paths and
autos in the app; this project runs the exported JSON and generated OpModes on
the robot.

JSON lives under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/brainstemPilotAuto/`
and is copied into APK assets on every build. Generated OpModes under
`opmodeAutos/` are not meant to be hand-edited.

- **App**: [brainstem-first.github.io/Brainstem-Pilot-UI](https://brainstem-first.github.io/Brainstem-Pilot-UI/)
- **Docs**: [Install + FTC guide](https://brainstem-first.github.io/Brainstem-Pilot-UI/docs/)
- **GitHub**: [BrainStem-FIRST/Brainstem-Pilot-UI](https://github.com/BrainStem-FIRST/Brainstem-Pilot-UI)

## Project layout

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── opmodes/                     TeleOp/Autonomous entry points
├── subsystems/drivetrain/       MecanumDrive, Pinpoint localizer
├── brainstemPilotAuto/          BrainSTEM Pilot JSON + generated autos
└── utils/
    ├── bezierCurveDrive/        Bézier path follower
    ├── pilotAutoBuilder/        Pilot runtime
    ├── math/
    └── misc/
```

## Where the build config lives

| File | What it holds |
| --- | --- |
| `settings.gradle` | Dairy Foundation repo (Sloth plugin) |
| `build.gradle` (root) | Sloth plugin version; Dairy + Road Runner repos |
| `TeamCode/build.gradle` | Added dependencies, Java 17, `syncBrainstemPilotAssets` |
| `build.dependencies.gradle` | FTC SDK artifacts only — do not add TeamCode libraries here |

`TeamCode` overrides `build.common.gradle` to Java 17 and `compileSdk 34`
because `PathFollowerUtils` uses a record. That does not change the API level
the robot runs.

## Updating for a new season

1. Merge the new SDK (do not hand-edit FTC versions):

   ```bash
   git fetch upstream
   git merge upstream/master
   ```

2. On conflicts in `build.gradle`, `settings.gradle`, and
   `TeamCode/build.gradle`, keep **both** sides. For `README.md`, keep ours:

   ```bash
   git checkout --ours README.md && git add README.md
   ```

   Upstream’s readme is always on
   [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController).

3. After the merge, check:

   - If `build.common.gradle` sets `compileSdk` above 34, drop the override in
     `TeamCode/build.gradle`.
   - If the wrapper jumps to Gradle 9, re-pin `8.14.5` unless Sloth already
     supports 9.
   - Bump Sloth, Slothboard, and Panels together (they share the `0.2.4+…`
     pin). The FTC merge does not update them.
   - Redraw Pilot field JSON for the new game. The Java is season-agnostic.
