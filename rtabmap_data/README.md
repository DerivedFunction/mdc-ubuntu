# RTAB-Map Quick Reference

## Run the Database Viewer
```bash
rtabmap-databaseViewer rtabmap.db
```

### What you can do inside:

* **3D Map View**: Visualize point clouds or meshes from your saved sessions.
* **Graph View**: See the pose graph (nodes = robot’s positions, edges = loop closures).
* **Images / Scans**: Inspect the raw RGB, depth, and laser scans stored in each node.
* **Export**: Save trajectories, point clouds, or meshes for RViz or other tools.

---

## Trim the Database

Keep only nodes **1 through 999**, removing later bad recordings:

```bash
rtabmap-reprocess -start 1 -stop 999 input.db output_trimmed.db
```

---

## Create a 2D Grid Map

```bash
rtabmap-reprocess -g2 input.db output.db
```

---

## Reprocess with Ground/Obstacle Height Filters

```bash
rtabmap-reprocess \
  --Grid/MinGroundHeight -0.25 \   # Too low -> ditches won't be obstacles
  --Grid/MaxGroundHeight 0.25 \   # Accept ground up to this height
  --Grid/MaxObstacleHeight 0.25 \ # Prevent ceiling from being marked as obstacle
  -g2 -db \
  input.db output.db
```