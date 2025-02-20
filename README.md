# Solverbot-work

# How to start
Instalar ROS2( Humble o Foxy) y todas sus dependencias.
Clonar este repositorio en un nuevo workspace.

### Build
```
colcon build
```
```
source install/setup.bash
```
### Dependencies
Revise el resto de los submodulos para ver cuales son las dependencias de cada uno e instale lo necesario.

### Launchers
Este proyecto usa varios launchers, la idea es ejecutar un launcher central y que este abra todos los demas.

```
ros2 launch motor_board_md49_ros2 main_launch.launch.py
```