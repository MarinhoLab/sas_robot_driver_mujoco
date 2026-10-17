# UMIRobot (MJCF)

The MuJoCo model of the [UMIRobot](https://mmmarinho.github.io/UMIRobot/), the open-{software, hardware} low-cost
robotic manipulator for education of Murilo M. Marinho ([arXiv:2301.06668](https://arxiv.org/abs/2301.06668)): five
revolute joints of hobby servos. The gripper (the sixth joint) is not included: the students design their own.

| File | Contents |
|---|---|
| `umirobot.xml` | The robot, with **position actuators** (as its servos): `ctrl` is the joint position. |
| `scene.xml` | `umirobot.xml` on a floor, with lights: the scene of the `scene` service of [`docker/compose.yaml`](../../docker/compose.yaml) (and to view the model, e.g., with MuJoCo's `simulate`). |
| `assets/` | The meshes (OBJ), one per color of each link. |

## Names

- Root body: `UMIRobot_link_1`, the base, fixed, at the origin of the robot. Bodies `UMIRobot_link_2` to
  `UMIRobot_link_6`, moved by the joints.
- Joints, in the order of the actuators (with the names of the CoppeliaSim scene of the UMIRobot): `UMIRobot_joint_1`
  to `UMIRobot_joint_5`, all in [-π/2, π/2] rad. Actuators of the same names.
- End effector: the site `end_effector`.
- Keyframe `home`: q = 0, the upper arm up and the forearm forward.

Total mass of the moving links: 0.22 kg.

## Kinematics

The bodies follow the DH parameters of the UMIRobot (`UMIRobotCSimRobot.kinematics()` in
`umirobot_control/commons/_umirobot_csim_robot.py` of [mmmarinho/umirobot](https://github.com/mmmarinho/umirobot)), so
that the pose of the site `end_effector` is their forward kinematics (to 1e-15), with q in the order of the joints:

| | joint 1 | joint 2 | joint 3 | joint 4 | joint 5 |
|---|---|---|---|---|---|
| θ (offset) | 0 | -90° | 0 | 0 | 0 |
| d (m) | 0.0245 | 0 | 0 | 0.16519 | 0 |
| a (m) | 0 | 0.0813 | 0 | 0 | 0 |
| α | -90° | 180° | 90° | -90° | 0 |

with the reference frame at joint 1 (`UMIRobot_reference_frame` of the URDF, which the CoppeliaSim scene gives to the
kinematics): in DQ Robotics,

```cpp
const DQ r = DQ(0.99863, 0, 0, -0.0523359).normalize();   // -6 degrees about z
robot.set_reference_frame(r + 0.5*E_*(-0.015469*i_ + 0.00323*j_ + 0.059873*k_)*r);
```

The meshes are placed in the bodies as in the URDF at q = 0. The URDF (CAD) differs slightly from the DH parameters (up
to about 1 cm along the axes and a few degrees, e.g., the axis of joint 5): as the joints move, the meshes may differ from
the kinematics by that much.

## Source

Made from [mmmarinho/umirobot](https://github.com/mmmarinho/umirobot), commit
`ef6dc8f8b800a303edc7b69728612019cfea656a`:

- `models/urdf/UMIRobot.urdf` (exported from CoppeliaSim) and its visual meshes (`*_vis_*.dae`): converted to OBJ,
  with the meshes of the same color of each link merged, and the colors of the URDF. The coordinates of the DAE files
  are already z-up (their `Y_UP` tag is ignored).
- The masses and inertias of the URDF, placed in the bodies of the DH parameters.
- The DH parameters of `umirobot_control/commons/_umirobot_csim_robot.py`.

Changes:

- The bodies follow the DH parameters (see above), and the site `end_effector` (DH frame 5) was added.
- The 210 convex collision meshes of the URDF are not included: the model has no collisions.
- `link5_navy.obj` decimated from 17808 to 4000 triangles (the other meshes are kept as they are, with normals), with
  `tools/decimate_meshes.py` of [cpp-interface-mujoco-robots](https://github.com/juanjqo/cpp-interface-mujoco-robots).
- Actuators (not in the URDF): position actuators within the torque limits of the joints, 1 N·m for joints 1 to 3 (a
  standard hobby servo: the URDF has 1000 N·m, the default of CoppeliaSim) and 0.324 N·m for joints 4 and 5 (from the
  URDF), with a stiffness of 10 and 2 N·m/rad. `armature` (the inertia of the motors through their gears) and
  `damping` were added.
- The keyframe `home`, and `scene.xml`.

## License

These files are released under the [GNU General Public License v3.0 or later](LICENSE), the license of
[mmmarinho/umirobot](https://github.com/mmmarinho/umirobot) from which they were made. Copyright (C) 2020-2023 Murilo
Marques Marinho (the UMIRobot, its URDF and meshes), and Copyright (C) 2026 Juan Jose Quiroz Omana (this MJCF model).
They are included here with the permission of Murilo M. Marinho. This applies to the files in this directory only.

The printable mechanical files of the UMIRobot on Thingiverse (not used here) are under CC BY-NC-ND 4.0, as stated on
its [website](https://mmmarinho.github.io/UMIRobot/).
