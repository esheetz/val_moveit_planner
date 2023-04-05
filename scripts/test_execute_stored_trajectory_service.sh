rosservice call /ValkyrieMoveItPlannerExecutorServerNode/execute_to_arm_goal "use_stored_robot_trajectory: true
planned_trajectory:
  joint_trajectory:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    joint_names:
    - ''
    points:
    - positions: [0]
      velocities: [0]
      accelerations: [0]
      effort: [0]
      time_from_start: {secs: 0, nsecs: 0}
  multi_dof_joint_trajectory:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    joint_names: ['']
    points:
    - transforms:
      - translation: {x: 0.0, y: 0.0, z: 0.0}
        rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
      velocities:
      - linear: {x: 0.0, y: 0.0, z: 0.0}
        angular: {x: 0.0, y: 0.0, z: 0.0}
      accelerations:
      - linear: {x: 0.0, y: 0.0, z: 0.0}
        angular: {x: 0.0, y: 0.0, z: 0.0}
      time_from_start: {secs: 0, nsecs: 0}"
