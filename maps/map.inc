# time to pause (in GUI mode) or quit (in headless mode (-g)) the simulation
quit_time 9999 # 1 hour of simulated time

# the resolution of the raytracing model
# Larger values speed up raytracing at the expense of fidelity in collision detection and sensing.
# see for details: http://playerstage.sourceforge.net/doc/stage-cvs/group__world.html
resolution 0.1

# default floor configurations, serve as a base to configure our world
# see for details: http://playerstage.sourceforge.net/doc/stage-cvs/group__model.html
define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  ranger_return 0.5
  laser_return 1

  obstacle_return 1
)
