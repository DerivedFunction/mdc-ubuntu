# RTAB-Map Quick Reference

This guide covers common RTAB-Map database operations such as viewing, trimming, and generating 2D maps.

---

## 📂 View a Database

**Command:**
```bash
rtabmap-databaseViewer rtabmap.db
```

**Description:**
Opens the graphical **Database Viewer** to inspect your `.db` file.

Inside Database Viewer, you can:

* **3D Map View** – Visualize point clouds or meshes.
* **Graph View** – See the pose graph (nodes = robot poses, edges = loop closures).
* **Images / Scans** – Browse RGB, depth, and laser scan data.
* **Export** – Save trajectories, meshes, or point clouds for RViz or external tools.

---

## ✂️ Trim a Database

**Command:**

```bash
rtabmap-reprocess -start 1 -stop 999 input.db output_trimmed.db
```

**Description:**
Keeps only nodes **1 through 999**, removing later nodes that may contain bad data.
This is useful when you want to discard sections of the map but keep earlier good data.

---

## 🗺️ Create a 2D Grid Map

**Command:**

```bash
rtabmap-reprocess -g2 input.db output.db
```

**Description:**
Generates a **2D occupancy grid** from a 3D database.
The result can be loaded into Nav2 (ROS2) using `map_server` for navigation.

---

## ⚙️ Reprocess with Ground/Obstacle Filters

**Command:**

```bash
rtabmap-reprocess \
  --Grid/MinGroundHeight -0.25 \
  --Grid/MaxGroundHeight 0.25 \
  --Grid/MaxObstacleHeight 0.25 \
  --Grid/DepthDecimation 1 \
  --Grid/RayTracing true \
  --Grid/MapFrameProjection true \
  -g2 -db \
  input.db output.db
```

**Description:**
Reprocesses the database with **height-based filtering** to improve 2D occupancy grids.

* `--Grid/MinGroundHeight` → Ignore points below this (avoid pits being obstacles).
* `--Grid/MaxGroundHeight` → Accept ground points up to this height.
* `--Grid/MaxObstacleHeight` → Ignore obstacles taller than this (avoid ceilings).
* `--Grid/DepthDecimation` → Downsample depth image (1 = full resolution, 2 = half, etc.).
* `--Grid/RayTracing` → Clears free space and handles dynamic obstacles.
* `--Grid/MapFrameProjection` → Project map in global frame instead of local.
* `-g2 -db` → Regenerates the 2D occupancy grid and updates the database.

---

## 🔑 Notes

* Always **back up** your `.db` before trimming or reprocessing.
* `rtabmap-reprocess` creates a new file (`output.db`) and does not overwrite the input.
* Use `map_server` in ROS2 to publish the generated 2D map for Nav2 navigation.