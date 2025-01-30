# MotorBoard MD49

### Build
```
colcon build --packages-select motor_board_md49_ros2
```
```
source install/setup.bash
```
### Dependencies
```
sudo apt install ros-humble-teleop-twist-keyboard
```
```
pip install pyserial
```

### Launchers
```
ros2 launch motor_board_md49_ros2 motor_board.launch.py
```
```
ros2 launch motor_board_md49_ros2 teleop.launch.py

```
Cuando se ejecute, se abrirá una terminal adicional con las instrucciones de teleoperación. Los comandos típicos son:

```
w: Avanzar.
s: Retroceder.
a: Girar a la izquierda.
d: Girar a la derecha.
x: Detenerse.
q / e: Incrementar / disminuir velocidad lineal.
z / c: Incrementar / disminuir velocidad angular.
```
### Pkg List
```
ros2 pkg list | grep motor_board_md49_ros2
```