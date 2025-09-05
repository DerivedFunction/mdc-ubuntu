# Run the db viewer
# What you can do inside:
# 3D Map View: Visualize point clouds or meshes from your saved sessions.
# Graph View: See the pose graph (nodes are your robot’s positions, edges are loop closures).
# Images / Scans: Inspect the raw RGB, depth, and laser scans stored in each node.
# Export: You can export trajectories, point clouds, or meshes for use in RViz or other tools.
# rtabmap-databaseViewer rtabmap.db


# Trim RTAB-Map database so that only nodes #1 through #999 are kept, 
# and everything after that (bad recording) is removed.
# rtabmap-reprocess -start 1 -stop 999 input.db output_trimmed.db

# Create a 2d grid
# rtabmap-reprocess -g2 input.db output.db 

