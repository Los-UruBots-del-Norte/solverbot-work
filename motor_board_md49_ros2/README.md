# MotorBoard MD49

```
colcon build --packages-select motor_board_md49_ros2
```
```
source install/setup.bash
```
```
sudo apt install ros-humble-teleop-twist-keyboard
```
```
ros2 launch motor_board_md49_ros2 motor_board.launch.py
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