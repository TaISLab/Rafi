# RAFI
Aportaciones del TFG de Rodrigo Castro Ochoa: Desarrollo de una interfaz software para el control de un manipulador móvil con ruedas omnidireccionales

## Red inalámbrica del laboratorio 3.506LII:
Credenciales de acceso:
SSID: ASUS_RAFI
Password: RED_24%rco

Administración del router:
usuario: admin
contraseña: RAFI2024_rco

Este router consta de un filtrado MAC en la red inalámbrica. Para añadir nuevos dispositivos, usar una conexión por cable Ethernet.

## Paquetes de ROS:

Controladores:
- `roboclaw_ros`. Versión modificada para su uso en RAFI. Disponible en: https://github.com/rodri-castro/roboclaw_ros.git
- `franka_ros_rafi`. Versión modificada de `franka_ros` para su uso en RAFI. Disponible en: https://github.com/rodri-castro/franka_ros_rafi.git

Teleoperación:
- `joy_base_control`. Teleoperación del controlador de la base. Disponible en: https://github.com/rodri-castro/joy_base_control.git
- `joy_franka_control`. Teleoperación del controlador de impedancia cartesiana del manipulador. Disponible en: https://github.com/rodri-castro/joy_franka_control.git
- `joy_franka_vel_control`. Teleoperación del controlador de velocidad cartesiana del manipulador. Disponible en: https://github.com/rodri-castro/joy_franka_vel_control.git
- `rafi_launch_files`. Paquete que contiene los archivos de lanzamiento necesarios para ejecutar los controladores y la teleoperación de forma simultánea. Disponible en: https://github.com/rodri-castro/rafi_launch_files.git

Otros:
- `franka_lock_unlock`. Desbloqueo de los frenos del manipulador y activación del FCI desde terminal. Disponible en: https://github.com/jk-ethz/franka_lock_unlock.git

