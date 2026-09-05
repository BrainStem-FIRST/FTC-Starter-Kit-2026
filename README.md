# FTC Starter Kit

An FTC robot controller project for the DECODE (2025-2026) season, built on FTC
SDK 11.2.1 with Sloth hot reload, FTC Dashboard, Panels, and Brainstem Pilot
wired in.

This is the stock FTC SDK project — `FtcRobotController` library module plus
`TeamCode` app module, Groovy Gradle scripts — with those tools added on top
rather than swapped in as a different project template. Everything added lives
in `TeamCode`; `FtcRobotController` is untouched, so season SDK updates can be
merged straight from upstream. See
[Updating for a new season](#updating-for-a-new-season).

For the FTC SDK's own documentation and release notes, see the
[upstream FtcRobotController repo](https://github.com/FIRST-Tech-Challenge/FtcRobotController),
and [ftc-docs](https://ftc-docs.firstinspires.org) for the Android Studio and
Blocks/OnBot Java tutorials.

## Features

- **Sloth hot-reload** — push code changes to the robot in under a second.
- **Panels** and **FTC Dashboard** — both supported side by side for live
  telemetry, tuning, and field visualization.
- **Brainstem Pilot** — JSON-driven autonomous routine builder, so autos can be
  assembled/edited without touching any code.
- Mecanum drive with a custom Bezier-curve path follower.

## Requirements

- **JDK 17 or later** to run the build. Android Studio's bundled JDK satisfies
  this; command-line builds need `JAVA_HOME` pointed at one.
- **Gradle 8.14.5**, pinned in `gradle/wrapper/gradle-wrapper.properties`. Do
  not bump this to Gradle 9 — see [Why Gradle 8](#why-gradle-8).

## Sloth

[Sloth](https://github.com/Dairy-Foundation/Sloth) is a hot-reload/OTA deploy
tool for FTC. Instead of building and installing a full APK every time you
change TeamCode, Sloth pushes just your changed classes to the robot -
typically in under a second, versus 30-60+ seconds for a full install.

Sloth only hot-reloads classes in `org.firstinspires.ftc.teamcode` (and
subpackages). Anything else - gradle files, new dependencies, non-TeamCode
code - requires a normal full install before Sloth can pick up further
changes correctly.

### One-time Android Studio setup

1. Set the run configuration's module: edit the `TeamCode` configuration,
   under "Module" pick the option that is **not** `<no module>`.
2. Do one full install using the `TeamCode` run configuration.
3. Use the bundled **Sloth Load** run configuration (`.run/Sloth Load.run.xml`,
   which runs `:TeamCode:deploySloth`) for fast iteration from then on.
4. Optionally, in the `TeamCode` run configuration, add
   `:TeamCode:removeSlothRemote` as a "Before launch" Gradle task ordered
   before the install step. The Sloth plugin already wires this in front of
   `installDebug`, so this is only needed if you install by some other route.
5. Fall back to the full `TeamCode` install whenever you change gradle files,
   add a dependency, or touch anything outside
   `org.firstinspires.ftc.teamcode`.

See the [Sloth README](https://github.com/Dairy-Foundation/Sloth#gradle-tasks)
for task screenshots if you get stuck.

### Deploying

Both the `TeamCode` install and `deploySloth` talk to the robot over `adb`, so
the Control Hub has to be connected first. `adb: no devices/emulators found`
means exactly that — nothing is attached.

- **USB**: plug into the Control Hub's USB-C port.
- **Wi-Fi**: join the robot's network, then
  `adb connect 192.168.43.1:5555` (Control Hub) or
  `adb connect 192.168.49.1:5555` (phone RC).

Confirm with `adb devices` before running any Gradle deploy task.

### Why Gradle 8

Gradle 9.0 removed the `Project.exec(Action)` API, which Sloth 0.2.4 (the
current release) still calls. On Gradle 9 every adb-backed Sloth task fails
immediately with:

```
Unable to find method 'org.gradle.process.ExecResult org.gradle.api.Project.exec(org.gradle.api.Action)'
```

Because the Sloth plugin wires `removeSlothRemote` in front of `installDebug`,
this breaks the ordinary Run button too, not just `deploySloth`. The wrapper is
therefore pinned to 8.14.5. Revisit only once Sloth ships a release that drops
the removed API.

## Panels + FTC Dashboard

Both [Panels](https://panels.bylazar.com) and FTC Dashboard are wired in, using
their Sloth-compatible forks (`com.bylazar.sloth:fullpanels` and
`com.acmerobotics.slothboard:dashboard`) so both hot-reload along with your
code. Do not add the non-Sloth versions of either alongside these — they clash
at package level.

For the same reason, `com.acmerobotics.roadrunner:ftc` and
`:actions` are declared with `exclude group: 'com.acmerobotics.dashboard'`,
since they pull in the vanilla Dashboard transitively.

- **Panels**: `http://192.168.43.1:8001` (Control Hub) or
  `http://192.168.49.1:8001` (Phone RC)
- **FTC Dashboard**: `http://192.168.43.1:8080/dash` (Control Hub) or
  `http://192.168.49.1:8080/dash` (Phone RC)

### Tunable values

Annotate a class with both `@Configurable` (Panels) and `@Config` (FTC
Dashboard) to make its `public static` fields tunable live from either UI -
they're separate systems, so both annotations are needed:

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

Route telemetry through `PanelsTelemetry` and a combined FTC `Telemetry`
(Driver Station + Dashboard) so one `.update()` call reaches all three
destinations:

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

(`PanelsTelemetry` is a Kotlin `object`, so from Java it's accessed via
`.INSTANCE`.)

## Brainstem Pilot

Brainstem Pilot lets you build autonomous routines from JSON assets (paths,
skeletons, variants) instead of writing them by hand in Java. The JSON files
live under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/brainstemPilotAuto/`
and are synced into `TeamCode/src/main/assets/brainstemPilotAuto/`
automatically before every build (see `syncBrainstemPilotAssets` in
`TeamCode/build.gradle`), where `BrainstemPilot` reads them
at runtime.

An autonomous OpMode is just a thin subclass of `PilotAutoBase` naming which
variant to run - see the generated example at
`brainstemPilotAuto/opmodeAutos/ExampleVariantAuto.java`:

```java
package org.firstinspires.ftc.teamcode.brainstemPilotAuto.opmodeAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.PilotAutoBase;

@Autonomous(name = "Example Variant Auto", group = "Pilot")
public class ExampleVariantAuto extends PilotAutoBase {
    public ExampleVariantAuto() {
        super("Example_Variant_Auto");
    }
}
```

`PilotAutoBase` handles setup (Dashboard telemetry, alliance selection,
`BrainstemPilot.initialize(...)`) and, on start, builds and runs the
`Action` sequence described by the named variant's JSON. Classes marked
`AUTO-GENERATED by Brainstem Pilot UI` under `opmodeAutos/` aren't meant to be
hand-edited - regenerate them from the JSON instead.

## Project layout

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── opmodes/                     TeleOp/Autonomous entry points
├── subsystems/drivetrain/       MecanumDrive, Pinpoint localizer
├── brainstemPilotAuto/          Brainstem Pilot JSON assets + generated autos
└── utils/
    ├── bezierCurveDrive/        Custom Bezier-curve path follower
    ├── pilotAutoBuilder/        Brainstem Pilot runtime (reads the JSON assets)
    ├── math/                    Geometry/math helpers
    └── misc/                    Alliance, telemetry, battery filtering, etc.
```

## Where the build config lives

| File | What it holds |
| --- | --- |
| `settings.gradle` | Dairy Foundation repo, for the Sloth plugin |
| `build.gradle` (root) | Sloth plugin version; Dairy + Road Runner repos |
| `TeamCode/build.gradle` | Sloth plugin, all added dependencies, Java 17 override, `syncBrainstemPilotAssets` |
| `build.dependencies.gradle` | FTC SDK artifacts only — shared with `FtcRobotController`, so don't add TeamCode-only libraries here |

`TeamCode` overrides `build.common.gradle`'s Java 8 / API 30 defaults to Java 17
and `compileSdk 34`, because `PathFollowerUtils` uses a record. This only
affects the language level of TeamCode's own code, not the API level the robot
runs.

## Updating for a new season

The repo has upstream configured, so a season SDK release is a merge, not a
rewrite:

```bash
git fetch upstream
git merge upstream/master
```

A season release bumps the FTC artifact versions in
`build.dependencies.gradle`, bumps `versionCode`/`versionName` in
`FtcRobotController/src/main/AndroidManifest.xml` (which `build.common.gradle`
scrapes to stamp the APK), swaps the sample OpModes, and sometimes moves the
AGP and Gradle wrapper versions — so hand-editing dependency coordinates isn't
enough.

Expect conflicts in `build.gradle`, `settings.gradle`, and
`TeamCode/build.gradle`; keep both sides in each. `README.md` also conflicts
whenever the SDK updates its release notes, since this file replaced the SDK's
readme at the same path — keep this one with
`git checkout --ours README.md && git add README.md` (upstream's is always
readable on
[their repo](https://github.com/FIRST-Tech-Challenge/FtcRobotController)). Then
check:

- If the new `build.common.gradle` sets `compileSdk` above 34, drop the
  override in `TeamCode/build.gradle` instead of letting it lower the value.
- If the merge bumps the Gradle wrapper to 9.x, re-pin it to 8.14.5 unless
  Sloth has been updated by then.
- Sloth, Slothboard, Panels, and Road Runner are unaffected by the FTC merge
  and need their own bumps. The three Sloth-flavored artifacts are cross-pinned
  through their `0.2.4+...` versions and must move in lockstep.
- Brainstem Pilot path JSON describes this season's field and is yours to
  redraw. The Java is season-agnostic — `FieldConstants` only does
  alliance/side mirroring.
