Models from https://github.com/google-deepmind/mujoco_menagerie
Commit: 8161bba264d7fa7c99ca301e91e7fb44737676ad

universal_robots_ur5e: BSD-3-Clause (LICENSE in that folder).
robotiq_2f85: BSD-2-Clause (LICENSE in that folder).
Both upstream folders are unmodified.
Both live under ur5e/ beside the assembly that uses them.
ur5e/ur5e_robotiq.xml adapts UR5e: relative mesh paths, attached 2F-85,
TCP and wrist camera, removed home keyframe (reset is provided by robot_core).
