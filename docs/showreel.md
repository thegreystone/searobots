# Showreel

The video in the [README](../README.md) is generated, not hand-cut. The showreel finds a
photogenic match with a good fight, plans an edit around what happens in it, and renders it
offline in the 3D viewer, one video frame at a time.

## Making one

Build the project, then run the showreel. It needs [ffmpeg](https://ffmpeg.org/) on the
`PATH`, in the `FFMPEG` environment variable, or passed with `--ffmpeg`
(`winget install ffmpeg`, `brew install ffmpeg`, `apt install ffmpeg`).

```bash
mvn clean install -DskipTests
mvn exec:exec -pl searobots-viewer -Dexec.mainClass=se.hirt.searobots.viewer.Showreel
```

or, from a packaged viewer jar (on macOS add `-XstartOnFirstThread`):

```bash
java -XX:+UseZGC -Xmx4g -jar searobots-viewer-<version>.jar --showreel [options]
```

Options are passed to the jar after `--showreel`. Through Maven, give the whole command line in
`exec.args`, for example
`-Dexec.args="-Xmx4g -cp %classpath se.hirt.searobots.viewer.Showreel --seed 574e4f1ec676d60a"`.

On Windows, `scripts\showreel.cmd` builds the project and launches the showreel from either Command
Prompt or PowerShell, forwarding any options below (add `--skip-build` to reuse an existing build
or `--clean` for a clean build):

```cmd
scripts\showreel.cmd
scripts\showreel.cmd --seed 574e4f1ec676d60a
```

A viewer window opens while the film renders; leave it alone until it closes. The whole run
takes a couple of minutes: about 40 seconds of scouting, then rendering at roughly 100 frames
a second on a desktop GPU. Rendering is paced by the recording, not the display, so a slower
machine produces the same film, only later.

The result, by default `target/showreel/searobots-showreel.mp4` (relative to the module when
run through Maven), is a 1920x960 (2:1), 60 fps H.264 film of about 85 seconds and 25-30 MB.
Next to it, `.plan.txt` lists the edit, and `showreel-work/` keeps the scouted replays with
their indexes.

| Option             | Default                       | Meaning                                                  |
|--------------------|-------------------------------|----------------------------------------------------------|
| `--out FILE`       | `target/showreel/…mp4`        | Video to write                                           |
| `--seed HEX`       |                               | Film this seed instead of scouting                       |
| `--replay FILE`    |                               | Film this `.srl` replay instead of scouting              |
| `--scout N`        | 64                            | Seeds whose terrain to survey                            |
| `--matches N`      | 16                            | Of those, how many to simulate                           |
| `--master HEX`     | `5eab07`                      | Seed for the scouting draw                               |
| `--length SECONDS` | 85                            | Target length; optional beats are dropped to fit         |
| `--size WxH`       | `1920x960`                    | Frame size                                               |
| `--fps N`          | 60                            | Frame rate                                               |
| `--crf N`          | 20                            | x264 quality (lower is better and larger)                |
| `--time HH:MM`     | `15:00`                       | Time of day, for the light                               |
| `--ffmpeg PATH`    |                               | ffmpeg executable                                        |
| `--frames DIR`     |                               | Write PNG frames instead of a video                      |
| `--plan-only`      |                               | Scout and plan, print the edit, render nothing           |

Matches are deterministic for the built-in controllers, so `--seed` reproduces a film exactly
as long as the controllers have not changed. When they do, the next scouting run picks
whatever is most watchable now. The current README film is seed `574e4f1ec676d60a`.

To update the README, post the new video as a comment on
[issue #1](https://github.com/thegreystone/searobots/issues/1), which keeps every README film
(drag the file into the comment box), and replace the `user-attachments` link in the README
with the one GitHub creates. The video is hosted by GitHub and never committed.

## How it works

1. **Scouting** (`ShowreelScout`). The terrain of many seeds is surveyed (`TerrainSurvey`):
   islands, their height and size, and how much of the seabed at submarine depth has 300 m
   cliffs. The most photogenic are simulated headless, Claude against Codex as in the viewer,
   with the match running on for 25 seconds after it is decided so the wreck can be seen going
   down (`SimulationLoop.setTicksAfterDecision`). Each match is recorded as a replay.
2. **Indexing** (`MatchIndex`). Each replay is indexed: launches, sonar pings, firing
   solutions, terminal approaches, closest passes, detonations, hits and deaths; and every half
   second, the scenery around each submarine (terrain relief, walls rising beside it, the
   seabed below, a nearby shore). Stretches with good scenery become scenic windows. The
   index is written beside the replay as `.index.txt`, so it can also be read to pick shots by
   hand. Matches are ranked by scenery and by how exciting the fight is (kills, hits, salvoes
   from both sides, near misses, fighting near an island).
3. **Planning** (`ShowreelPlan`). The edit is a list of beats. Each beat is a camera shot over
   a stretch of the match, with its own playback speed: time-lapse while the submarines hunt,
   real time for launches, slow motion into a detonation. Simulation time only moves forward,
   and a hard cut may skip ahead. Because the plan knows what is coming, every shot is in place
   before its event. The film opens with a title over a low flight towards an island, dives to
   the first submarine, introduces both where their scenery is best, follows the hunt from
   above, then the launches, hits and kill, and ends on the surface.
4. **Rendering** (`Showreel`). The viewer plays the replay paused, stepping it to exactly the
   planned tick for every video frame, and runs on a fixed time step instead of the wall
   clock. The Cinematic Director, in scripted mode, frames the cued shots. The finished frame,
   with titles and captions (`ShowreelOverlay`), is read back and streamed to ffmpeg
   (`FrameRecorder`). Frames are held back while island trees near the camera are still
   streaming in.

## Shots

Besides its own shots, the Director has a few that are only cued by the showreel:

- **Aerial** and **outro**: a low flight over the water towards an island, chosen so the
  camera does not look into the sun, and the same in reverse to end on.
- **Launch**: the camera waits just off the torpedoes' path ahead of the bow. The tube doors
  open, the weapon slides out and comes at the camera, and as it glides past, the camera takes
  up its speed and swings round it, close enough to read its markings.
- **Sonar**: well off a pinging submarine, in slow motion, as the pulse swells out of the hull
  and breaks the surface to spread across it as a widening circle. The camera orbits the
  submarine and descends as it goes: from a little above, looking down across it, to a little
  below, looking up across it at the surface. For a ping from deep water, which would not reach
  the surface during the shot, it stays above.
- **Scenic pass**: a submarine in front of the cliff or ridge its index entry points at.
- **Incoming**: over the target's shoulder towards the torpedo closing on it.
- **Torpedo chase** and **impact**: riding behind a torpedo, and over its shoulder at the hull
  it is about to hit, slowing down into the detonation.

In scripted mode the camera is kept out of hulls, the overview gets team-coloured markers and
bolder tracks, and the seabed is lit as if seen through water.
