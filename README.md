# :electric_plug: Serial Link Interfaces

This ROS2 package defines custom `.msg` and `.action` files for controlling serial link robot arms. It was created to be used alongside the [Serial Link Action Server](https://github.com/Woolfrey/server_serial_link) and [Serial Link Action Client](https://github.com/Woolfrey/client_serial_link) ROS2 packages. These interfaces have been deliberately abstracted for seemless integration with other potential action servers and action clients.

#### :sparkles: Features:
- Message types for defining both joint, and Cartesian trajectories for robot control.
- Statistics messages for summarising robot performance.
- Action definitions for real-time feedback control.

#### :compass: Navigation
- [Installation](#clipboard-installation)
- [Usage](#wrench-usage)
- [Release Notes](#package-release-notes---v100-april-2025)
- [Contributing](#handshake-contributing)
- [License](#scroll-license)

## :clipboard: Installation

Your directory structure should end up looking something like this:
```
ros2_workspace/
├── build/
├── install/
├── log/
└── src/
    └── interface_serial_link
        ├── msg/
        |   ├── CartesianState.msg
        |   ├── CartesianTrajectoryPoint.msg
        |   ├── JointCommand.msg
        |   ├── JointState.msg
        |   ├── JointTrajectoryPoint.msg
        |   └── Statistics.msg
        ├── action/
        |   ├── FollowTransform.action
        |   ├── FollowTwist.action
        |   ├── TrackCartesianTrajectory.action
        |   └── TrackJointTrajectory.action
        ├── CMakeLists.txt
        ├── package.xml
        └── README.md
```

1. In your ROS2 workspace `<your_workspace>/src/` directory, clone the package:

```
git clone https://github.com/Woolfrey/interface_serial_link.git
```

> [!NOTE]
> The repository name is `interface_serial_link`, but the project name in the CMakeLists.txt and package.xml file is `serial_link_interfaces`. I did this deliberately so it's easier to sort through my repositories on github.

2. Navigate back to the root `<your_workspace>` and install:

```
colcon build --packages-select --serial_link_interfaces
```

3. Source the changes (if you haven't modified your `.bashrc` file):

```
source ./install/setup.bash
```

4. Validate the installation:

```
ros2 interface list
```

Scroll down and you should see the following:

<p align="center">
  <img src="doc/ros2_interface_list.png" width="600" height="auto"/>
</p>

[:top: Back to Top.](#electric_plug-serial-link-interfaces)

## :wrench: Usage

To use the interfaces, you can include them in your `.h`, `.hpp`, and/or `.cpp` files:

```
#include <serial_link_interfaces/msg/joint_trajectory_point.hpp>
#include <serial_link_interfaces/action/track_joint_trajectory.hpp>
```

In your `CMakeLists.txt` file you must tell the compiler to find the package:
```
find_package(serial_link_interfaces REQUIRED)
```
and list them as dependencies in the `package.xml` file:
```
<build_depend>serial_link_interfaces</build_depend>
<exec_depend>serial_link_interfaces</exec_depend>
```

For more details on how to implement them, check out:
- My [ROS2 Tutorials](https://github.com/Woolfrey/tutorial_ros2) on how to create  subscribers & publishers, services, and action servers.
- My [Serial Link Action Server](https://github.com/Woolfrey/server_serial_link) package, and
- My [Serial Link Action Client](https://github.com/Woolfrey/client_serial_link) packge.

[:top: Back to Top.](#electric_plug-serial-link-interfaces)

## :package: Release Notes - v1.0.0 (April 2025)

- Messages:
  - CartesianState
  - CartesianTrajectoryPoint
  - JointCommand
  - JointState
  - JointTrajectoryPoint
  - Statistics
- Actions
  - FollowTransform
  - FollowTwist
  - TrackCartesianTrajectory
  - TrackJointTrajectory
 
[:top: Back to Top.](#electric_plug-serial-link-interfaces)

## :handshake: Contributing

Contributions are always welcome. Feel free to fork the repository, make changes, and issue a pull request.

You can also raise an issue asking for new features.

[:top: Back to Top.](#electric_plug-serial-link-interfaces)

## :scroll: License

This software package is licensed under the [GNU General Public License v3.0 (GPL-3.0)](https://choosealicense.com/licenses/gpl-3.0/). You are free to use, modify, and distribute this package, provided that any modified versions also comply with the GPL-3.0 license. All modified versions must make the source code available and be licensed under GPL-3.0. The license also ensures that the software remains free and prohibits the use of proprietary restrictions such as Digital Rights Management (DRM) and patent claims. For more details, please refer to the [full license text](LICENSE).

[:top: Back to Top.](#electric_plug-serial-link-interfaces)
