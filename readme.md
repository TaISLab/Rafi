# RAFI

## Red inalámbrica del laboratorio 3.506LII:
Credenciales de acceso:
SSID: ASUS_RAFI
Password: RED_24%rco

Administración del router:
usuario: admin
contraseña: RAFI2024_rco

Este router consta de un filtrado MAC en la red inalámbrica. Para añadir nuevos dispositivos, usar una conexión por cable Ethernet.


## Desarrollo de una interfaz software para el control de un manipulador móvil con ruedas omnidireccionales

Se trata del TFG realizado por Rodrigo Castro Ochoa.

## Software desarrollado:

Se han implementado paquetes de ROS encargados del control, teleoperación y gestión del manipulador. Los enlaces se encuentran a continuación.

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

## Experimentos realizados

Para comprobar el correcto funcionamiento de la interfaz se ha llevado a cabo distintos experimientos.

- Experimento 1. Preparación del manipulador para el movimiento mediante terminal. Este experimento involucra al paquete `franka_lock_unlock`.
- Experimento 2. Teleoperación del controlador de la base. Este experimento involucra los paquetes `roboclaw_ros` y `joy_base_control`.
- Experimento 3. Teleoperación del controlador de impedancia cartesiana del manipulador. Este experimento involucra los paquetes `franka_ros_rafi` y `joy_franka_control`.
- Experimento 4. Teleoperación del controlador de velocidad cartesiana del manipulador. Exte experimento requiere los paquetes `franka_ros_rafi` y `joy_franka_vel_control`.
- Experimento 5. Teleoperación del Esquema A de RAFI. El esquema A consiste en la ejecución simultánea del controlador de la base y el controlador de impedancia cartesiana del manipulador. Requiere los paquetes `roboclaw_ros`, `franka_ros_rafi` y `joy_franka_control`.
- Experimento 6. Teleoperación del Esquema B de RAFI. El esquema B consiste en la ejecución simultánea del controlador de la base y el controlador de velocidad cartesiana del manipulador. Requiere los paquetes `roboclaw_ros`, `franka_ros_vel_rafi` y `joy_franka_control`.

El paquete `rafi_launch_files` contiene los archivos de lanzamiento de los distintos experimentos.

El vídeo recopilatorio de los experimentos se encuentra disponible en: https://youtu.be/dym2i6fTFeE
