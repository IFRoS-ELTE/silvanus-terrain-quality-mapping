# Project plan, October to end of November

Terrain quality and hazard mapping with Silvanus. IFRoS Robotics Lab, ELTE.

**Draft for discussion.** Agree it as a team, then it becomes the plan.

Eight weeks from 1 October. Final delivery is a **presentation**, end of November.

---

## What finished looks like

The robot covers an area on its own in a zigzag sweep. While it drives, the
camera measures the ground ahead and the IMU measures what the wheels actually
felt. Everything lands on a map. The robot slows down before hazards.



---

## How a hazard is classified

Every hazard ends up in one of three classes, and this is the headline result.

| Class | Camera saw it | IMU felt it | Meaning | Colour on the map |
|---|---|---|---|---|
| **A** | yes | no | Seen but not confirmed. Either the wheels missed it, or it is too small for this robot to feel. | **yellow** |
| **B** | no | yes | Felt but not seen. The camera missed it: too small, masked, or blurred. | **blue** |
| **C** | yes | yes | **Confirmed.** Both agree. | **red** |

Class C is what the project is for. Classes A and B are where it is interesting:
A tells us the smallest hazard this robot actually feels, B tells us what the
camera misses. Both are results, not failures.

---

## How it is shown

Three levels, for three different purposes.

### Level 1 — live, while driving

**Point cloud coloured by height above the ground plane.** Every depth point
already has a signed distance to the fitted plane; that becomes a colour:

```
below the plane  ->  BLUE    hole
on the plane     ->  GREY    flat ground
above the plane  ->  RED     bump
```

Full scale at plus or minus 40 mm. A book glows red, a pothole glows blue, flat
floor stays grey. Diverging colours, because the data has a meaningful zero:
the ground plane itself.

Hazard markers alongside it: a box per hazard, sized to the measured length,
width and height, labelled `bump 54 mm`.

### Level 2 — the map layer

Three things on top of the 2D map from SLAM:

- **Driven path coloured by roughness.** One hue going light to dark, not a
  rainbow. Roughness is an amount, and rainbows make amounts look like
  categories.
- **Terrain class per 0.5 m cell.** Distinct colours per surface: tile,
  concrete, asphalt, grass, gravel, dirt. Fixed colour per surface so adding one
  never repaints the others.
- **Hazard markers, numbered**, coloured A yellow, B blue, C red as above, so
  they match the table in the presentation.

### Level 3 — the presentation

- The map figure: path coloured by roughness, numbered hazard pins
- The hazard table, one row per hazard, with an image crop
- Roughness per surface, box plots
- **Camera-predicted unevenness against IMU-measured roughness**, one point per
  0.25 m of path. If that correlates, the project's central claim is proven in a
  single figure.

---

## Week by week

### Week 1 — 1 to 7 October. Everything on real data.

- Depth pipeline running as a ROS 2 node, publishing `/ground/hazards`,
  `/ground/hazard_markers`, `/ground/unevenness`
- `xsens_link` to `base_link` transform measured and published
- **First outdoor bag**, with a written surface log: one pass over each surface
  we can reach, times and surface types noted as we drive
- Crack model running on real frames from our own bags, not simulation
- Robot body and gripper mask
- Topic names and record schema agreed, written down, not changed after

**Gate:** everyone working on real data from our own robot, and we know what
outdoors actually looks like.

The first outdoor bag is deliberately early. It does not need to be good: its
job is to surface the problems while there is still time to solve them. Expect
at least these, and write down what we find:

- bright sunlight washes out the D435's depth
- GNSS needs several minutes outdoors to get a fix
- 2D lidar has little to match against in an open garden, so SLAM may not hold
- grass makes skid-steer wheel odometry slip

The full labelled dataset comes in week 4. This is the reconnaissance run.

### Week 2 — 8 to 14 October. Localization and IMU.

- Indoor map built with SLAM Toolbox and saved; AMCL localizing on it
- IMU roughness node: gravity removal, 1 s windows, RMS, peak, FFT bands
- Nav2 configured for the Scout: footprint, inflation, controller
- **Outdoor localization tested** on the week 1 bag: does SLAM hold, is the GNSS
  fix good enough, how far does wheel odometry drift on grass. Pick one and
  write down why.
- RGB terrain classifier, first version
- Annotation under way on frames from our own bags

**Gate:** the robot drives to a goal on a saved map, by itself.

### Week 3 — 15 to 21 October. Coverage and fusion.

- Boustrophedon waypoint generator; robot sweeps a rectangle indoors
- Speed governor between the velocity command and the base
- Vibration-based terrain classifier trained on labelled segments
- Fusion node joining camera and IMU into one record every 0.25 m
- **Hazard classes A, B and C assigned and stored**

**Gate:** autonomous coverage of a hall, recording throughout.

### Week 4 — 22 to 28 October. Outdoor data.

- Outdoor localization decided and tested: GPS, live SLAM, or odometry.
  Written up with numbers.
- Labelled outdoor runs: pavement, grass, gravel, dirt, slope. Two or three
  repeats of each, with a written log of times and surface types.
- EKF fusing wheel odometry, IMU and GPS
- Terrain classifier trained on outdoor images
- Map layer: path coloured by roughness, hazard markers, terrain grid

**Gate:** a labelled outdoor dataset exists. **This is the hardest deadline in
the plan.** Without it, terrain classification has no results at all.

### Week 5 — 29 October to 4 November. Everything together.

- Full pipeline on one bag, on one laptop
- Then the same, live on the robot
- Book demo: robot approaches, slows by itself, crosses, IMU spike logged,
  marker appears on the map in class C
- Live view showing map, path, hazards and camera

**Gate:** the headline demo works end to end. **If it does not work this week,
cut scope rather than pushing it later.**

### Week 6 — 5 to 11 November. Results.

- Three repeats of each route
- Roughness versus speed over one stretch at three speeds
- Roughness per surface, box plots
- **Camera-predicted unevenness against IMU-measured roughness**
- Counts of hazard classes A, B and C per route
- Smallest hazard the robot actually feels, from the crossing trials
- Crack segmentation results: Dice, IoU, false positives per metre

**Gate:** every figure that goes in the presentation exists, even if rough.

### Week 7 — 12 to 18 November. Presentation and video.

- Presentation drafted, every slide present
- Demo video: indoor book run and outdoor terrain run
- Any run whose data turned out weak, recorded again

**Gate:** a complete draft deck exists. Nothing left to make in the last week.

### Week 8 — 19 to 25 November. Finish.

- Final presentation
- Demo video edited
- Short guide so the next group can run the system
- Repository tidy: README, interface contracts, how to reproduce every figure

**No new features this week.** Only finishing. The few days to the end of
November are buffer, not working time.

---

## Stretch, only if weeks 1 to 5 land on time

- Terrain costmap layer, so the planner avoids rough ground
- Change detection between repeated runs
- Nav2 curving around obstacles instead of stopping and waiting

---

## What could sink this

| Risk | What we do |
|---|---|
| **IMU stuck at 10 Hz** | Roughness needs 100 Hz or more. Everything else proceeds; only the numbers wait. |
| **No labelled outdoor data by week 4** | Terrain classification then has no results. Hardest deadline in the plan. |
| **Camera USB dropouts** | Check the camera rate immediately before every recording. Never record blind. |
| **Robot storage** | Record 90 s, copy off, delete. |
| **Battery** | Check `/scout_status` voltage at the start of every session. |
| **Robot time** | Only one person needs the robot. Everyone else works from bags. |
| **Work overlapping** | Say who is doing what at the weekly sync, before starting. |

---

## How we work

**Bags live on the shared drive, not in git.** GitHub rejects files over 100 MB
and ours are about 1.7 GB each.

https://drive.google.com/drive/folders/1jg45KSiKPr-DoeFKEp3v48fnMJFQSeWJ?usp=drive_link

See `docs/DATA.md` for what is there, how to replay it, and how to pull frames
out for annotation.

**One branch per person**, named after them. Merge into `main` through a pull
request.

**Topic names and the record schema are fixed in week 1** and not changed after.
That is what lets three people build separately and have it join up.

**Weekly 30 minute sync.** What landed, what is blocked, what changes.
