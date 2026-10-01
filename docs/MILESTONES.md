# Project plan to the end of November

Terrain quality and hazard mapping with Silvanus. IFRoS Robotics Lab, ELTE.

**Draft for discussion.** Agree it as a team, then it becomes the plan.

Eight weeks, from 6 October to 30 November.

---

## What "finished" means

A demo and a report showing:

1. The robot covers an area on its own, in a zigzag sweep.
2. While it drives, the camera measures the ground ahead: holes, bumps,
   unevenness, surface type.
3. The IMU measures what the wheels actually felt.
4. Everything lands on a map: the path coloured by roughness, hazard markers
   with measured sizes, terrain type per area.
5. The robot slows down before hazards.
6. A report that comes out of the data automatically.

The one-line story: **the camera predicts, the IMU confirms.**

---

## Who does what

Agreed by the three of us. Fill in names where they are open.

| Area | Owner |
|---|---|
| Robot operation, bring-up, recording sessions | Praveen |
| Depth geometry: ground plane, holes and bumps | Praveen |
| IMU: transform, time sync, roughness, vibration-based terrain class | Praveen |
| Localization, Nav2, coverage path planning, speed governor | Praveen |
| Crack segmentation: dataset, training, deployment | Aritra |
| RGB terrain classification from image patches | open |
| Frame annotation from our own bags | open |
| Fusion node, record schema, the map layer | open |
| Dashboard, report generator, evaluation plots | open |

The four open items matter. The ones most likely to be left too late are the
**IMU signal side** and the **report generator**, because neither is urgent
until suddenly it is.

---

## Week by week

### Week 1 — 6 to 12 October. Everything on real data.

| Who | Deliverable |
|---|---|
| Praveen | Depth pipeline running as a ROS 2 node, publishing `/ground/hazards`, `/ground/hazard_markers`, `/ground/unevenness`. Tested against the indoor bags. |
| Praveen | `xsens_link` to `base_link` transform measured and published. |
| Praveen | Second outdoor attempt, with a **surface log** (times and surface types). |
| Vision | Crack model run on real frames from our bags, not simulation. |
| Vision | Robot body and gripper mask. |
| All | 30 minute call: agree the topic names and the record schema. |

**Gate:** everyone is working on real data from our own robot by Friday.

### Week 2 — 13 to 19 October. Localization.

| Who | Deliverable |
|---|---|
| Praveen | Indoor map built with SLAM Toolbox and saved. AMCL localizing on it. |
| Praveen | IMU roughness node: gravity removal, 1 s windows, RMS, peak, FFT bands. |
| Praveen | Nav2 configured for the Scout: footprint, inflation, controller. |
| Vision | RGB terrain classifier, first version. |
| Vision | Annotation under way on frames from our own bags. |
| Fusion | Record schema fixed and documented. |

**Gate:** the robot drives to a goal on a saved map, by itself.

### Week 3 — 20 to 26 October. Coverage.

| Who | Deliverable |
|---|---|
| Praveen | Boustrophedon waypoint generator. Robot sweeps a rectangle indoors. |
| Praveen | Speed governor between the velocity command and the base. |
| Praveen | Vibration-based terrain classifier, trained on labelled segments. |
| Vision | Crack model fine tuned on our own annotated frames. |
| Fusion | Fusion node joining camera and IMU into records every 0.25 m. |

**Gate:** autonomous coverage of a hall, with data recorded throughout.

### Week 4 — 27 October to 2 November. Outdoor data.

| Who | Deliverable |
|---|---|
| Praveen | Outdoor localization decided and tested: GPS, live SLAM, or odometry. Written up with numbers. |
| Praveen | Labelled outdoor runs: pavement, grass, gravel, dirt, slope. Two or three repeats each. |
| Praveen | EKF fusing wheel odometry, IMU and GPS. |
| Vision | Terrain classifier trained on outdoor images. |
| Fusion | Map layer: path coloured by roughness, hazard markers, terrain grid. |

**Gate:** a labelled outdoor dataset exists. This is the one that gates every result.

### Week 5 — 3 to 9 November. Everything together.

| Who | Deliverable |
|---|---|
| All | Full pipeline running on one bag, on one laptop. |
| All | Then the same, live on the robot. |
| Praveen | Book demo: robot approaches, slows by itself, crosses, IMU spike logged, marker appears on the map. |
| Fusion | Dashboard showing map, path, hazards and camera live. |

**Gate:** the headline demo works end to end. **If it does not work this week,
cut scope rather than pushing it later.**

### Week 6 — 10 to 16 November. Results.

| Who | Deliverable |
|---|---|
| All | Evaluation runs: three repeats of each route. |
| Praveen | Roughness versus speed over the same stretch at three speeds. |
| Fusion | Roughness per surface, box plots. |
| Fusion | **Camera-predicted unevenness against IMU-measured roughness.** This single figure is the project's central claim. |
| Vision | Crack segmentation results: Dice, IoU, false positives per metre. |

**Gate:** every figure that goes in the report exists, even if rough.

### Week 7 — 17 to 23 November. Writing.

| Who | Deliverable |
|---|---|
| All | Report draft, complete, all sections. |
| Fusion | Report generator producing the inspection report automatically. |
| Praveen | Demo video: indoor book run and outdoor terrain run. |
| All | Repeat any run whose data turned out to be weak. |

**Gate:** a complete draft exists. Nothing is left to be written in the last week.

### Week 8 — 24 to 30 November. Finish.

| Who | Deliverable |
|---|---|
| All | Final report. |
| All | Presentation. |
| All | Demo video edited. |
| Praveen | Short guide so the next group can run the system. |
| All | Repository tidy: README, interface contracts, how to reproduce every figure. |

**No new features this week.** Only finishing.

---

## Stretch, only if weeks 1 to 5 land on time

- Terrain costmap layer, so the planner avoids rough ground
- Change detection between repeated runs
- Nav2 curving around obstacles instead of stopping and waiting

---

## What could sink this, and what we do about it

| Risk | What we do |
|---|---|
| **IMU stuck at 10 Hz** | Raised with the professor. Roughness needs 100 Hz or more. Everything else proceeds meanwhile; only the numbers wait. |
| **No labelled outdoor data by week 4** | Then terrain classification has no results. This is the hardest deadline in the plan. |
| **Camera USB dropouts** | Check the camera rate immediately before every recording. Never record blind. |
| **Robot storage, 12 GB free** | Record 90 s at a time, copy off, delete. Asked for an external SSD. |
| **Battery** | Check `/scout_status` voltage at the start of every session. |
| **Robot time** | Only one of us needs the robot. Everyone else works from bags. |
| **Work overlapping** | The table above. Revisit it if anyone starts something not on their row. |

---

## How we work

**Bags live on the shared drive, not in git.** GitHub rejects files over 100 MB
and our bags are about 1.7 GB each. The repo holds code, configs and documents.

Shared drive:
https://drive.google.com/drive/folders/1jg45KSiKPr-DoeFKEp3v48fnMJFQSeWJ?usp=drive_link

See `docs/DATA.md` for what is there, how to replay it, and how to pull frames
out for annotation.

**One branch per person**, named after them: `praveen-robot-and-depth`,
`aritra-crack-detection`. Merge into `main` through a pull request.

**Topic names and the record schema are fixed in week 1** and then nobody
changes them without telling the others. That is what lets three people build
separately and have it work when joined.

**Weekly 30 minute sync.** What landed, what is blocked, what changes.
