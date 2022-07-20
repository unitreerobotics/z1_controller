# v3.6.1
The z1_controller is mainly used to control the robot arm and communicate with the Z1 SDK

## Notice
support robot: Z1(v3.6.1)
not support robot: Z1(v3.4, v3.5)

## Dependencies
- [ROS](https://www.ros.org/)（Melodic）
- build-essential

```bash
sudo apt install build-essential
```

- Boost (version 1.5.4 or higher)

```bash
dpkg -S /usr/include/boost/version.hpp	# check boost version
sudo apt install libboost-dev			# install boost
```

- CMake (version 2.8.3 or higher)

```bash
cmake --version 		# check cmake version
sudo apt install cmake	# install cmake
```

- [Eigen](https://gitlab.com/libeigen/eigen/-/releases/3.3.9) (version 3.3.9 or higher)

```bash
cd eigen-3.3.9
mkdir build && cd build
cmake ..
sudo make install
sudo ln -s /usr/local/include/eigen3  /usr/include/eigen3
sudo ln -s /usr/local/include/eigen3/Eigen  /usr/local/include/Eigen
```

- [RBDL](https://github.com/rbdl/rbdl/releases/tag/v2.6.0) (version 2.6.0)

```bash
cd rbdl-2.6.0
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
sudo make install
sudo sh -c "echo '/usr/local/lib' >> /etc/ld.so.conf"
sudo ldconfig
```

## Build

```bash
# config /<path to>/z1_controller/CMakeList.txt
cd /<path to>/z1_controller && mkdir build && cd build
cmake ..
make -j4
```

## Usage

The default IP of the robot is 192.168.123.110, you need to change the IP of your PC before using the SDK so that your PC can ping to the robot.

First, please connect the network cable between your PC and robot. Then run ifconfig in a terminal, you will find your port name. For example, enpxxx.

```bash
sudo ifconfig enpxxx down  # enpxxx is your PC port
sudo ifconfig enpxxx 192.168.123.162/24
sudo ifconfig enpxxx up
ping 192.168.123.110
```

Then run program.

```
cd /<path to>/z1_controller/build
sudo ./z1_ctrl
```

- FSM(finite-state machine)

| State       | Keyswitch | Switchable                  |
| ----------- | --------- | --------------------------- |
| BACKTOSTART | ～        | 1 2                         |
| PASSIVE     | 1         | ~ 2 3 =                     |
| JOINTCTRL   | 2         | ~ 1 3 4 5 6 7 8 9 0 -       |
| CARTESIAN   | 3         | ~ 1 2 4 5 6 9               |
| MoveJ       | 4         | ~ 1 2 3 5 6 9               |
| MoveL       | 5         | ~ 1 2 3 4 6 9               |
| MOVEC       | 6         | ~ 1 2 3 4 5 9               |
| TEACH       | 7         | ~ 1 2                       |
| TEACHREPEAT | 8         | automatically switches to 2 |
| SAVESTATE   | 9         | automatically switches to 2 |
| TOSTATE     | 0         | automatically switches to 2 |
| TRAJECTORY  | -         | ~ 1 2                       |
| CALIBRATION | =         | automatically switches to 2 |
| NEXT        | ]         | next state                  |

#### FSM Details

- Key ~ (BACKTOSTART) : All motors return to initial positions

- Key1（PASSIVE）: All motor enter passive state(The state of the Z1 startup)

- Key2（ JOINTCTRL）

  | Joint ID                        | 0                       | 1                       | 2                       | 3                       | 4                       | 5                       | Gripper                 |
  | :------------------------------ | :---------------------- | :---------------------- | :---------------------- | :---------------------- | :---------------------- | :---------------------- | ----------------------- |
  | Keyboard                        | Q/A                     | W/ S                    | D/ E                    | R/F                     | T/G                     | Y/H                     | up/down                 |
  | Joint Action <br />(right hand) | positive<br />/negative | positive<br />/negative | positive<br />/negative | positive<br />/negative | positive<br />/negative | positive<br />/negative | positive<br />/negative |

- Key3（CARTESIAN）: The reference coordinate system is cartesian

  | Keyboard     | Q/A                    | W/S              | E/D           | R/F                     | T/G                      | Y/H                    |
  | ------------ | ---------------------- | ---------------- | ------------- | ----------------------- | ------------------------ | ---------------------- |
  | Key Function | forward<br />/backward | right<br />/left | up<br />/down | roll <br />(right hand) | pitch<br /> (right hand) | yaw<br /> (right hand) |

- Key4（MoveJ）:

  ```
  Key4—— Enter the desired end pose（roll pitch yaw x y z）——The Z1 joint rotate to the joint target point（After Z1 arrived the target point, it automatically switches to the joint space control state）
  ```

- Key5（MoveL）:

  ```
  Key5——Enter the desired end pose（roll pitch yaw x y z）——The Z1 follows the generated straight trajectory to the target point（After Z1 arrived the target point, it automatically switches to the joint space control state）
  ```

- Key6（MoveC）:

  ```
  Key6——Enter the desired middle and end pose（roll pitch yaw x y z）——The Z1 follows the generated arc trajectory to the target point（After Z1 arrived the target point, it automatically switches to the joint space control state）
  ```

- Key7（TEACH）:

  ```
  Key7——Enter the teaching trajectory label —— Drag Z1 —— Press Key2 to complete teaching.
  ```

- Key8（TEACHREPEAT）:

  ```
  Key8———— Enter the saved teaching trajectory label—— Z1 repeate the teaching trajectory
  ```

- Key9（SAVESTATE）:

  ```
  Key9——Enter the current pose label —— Z1 automatically switches to the joint space control state
  ```

- Key0（TOSTATE）:

  ```
  Key0——Enter the pose label to save（After Z1 arrived the target point, it automatically switches to the joint space control state）
  ```

- Key -（TRAJECTORY）:

  ```
  Key - —— Z1 repeats in a written trajectory
  ```

- Key=（CALIBRATION)；

  ```
  Key= —— Set the current position as the initial position, and enter the joint space control state after setting
  ```

- Key ] （NEXT）: used to debug joystick contrl

  ```
  Key]—— Enter next state
  ```

## SDK

If you want to develop your own control methods of Z1, you can  refer to the SDK(z1_sdk).

We have written an example of keyboard control based on SDK, you can use it fllowing the steps below.

### State change

- First, set(CTRL_PANEL SDK)  # z1_ws/src/z1_controller/CMakeList.txt，and then rebuild the z1_ws to generate z1_ctrl, then open a teminal to run z1_ctrl

  ```
  cd /<path to>/z1_controller/build
  sudo ./z1_ctrl
  ```

- Sencond, build the z1_sdk, and then open another terminal to run example.

  ```
  cd /<path to>/z1_sdk && mkdir build && cd build
  cmake ..
  make -j4
  ./example_state_send
  ```

### Low level control

```
sudo ./z1_ctrl			# Running in a terminal
./example_lowCmd_send	# Running in another terminal
```

### Keyboard control

```
sudo ./z1_ctrl				# Running in a terminal
./example_keyboard_send		# Running in another terminal
```

### State control

```
sudo ./z1_ctrl				# Running in a terminal
./example_keyboard_send		# Running in another terminal
```

