
#Naranjelio Workspace
Este repositorio contiene el espacio de trabajo de ROS 2 para el desarrollo y control del robot cuadrúpedo Naranjelio. El sistema está diseñado para gestionar la cinemática, el control de actuadores y la simulación del robot.
Este es el repositorio en el cual se tiene la informacion previa del robot, se busca seguir trabajando en este repositorio para migrarlo a un ambiente en el cual nos sea favorable para continuar con el desarrollo de difrentes proyectos
##Requisitos del Sistema
Para asegurar la compatibilidad, el entorno debe cumplir con las siguientes especificaciones:

Sistema Operativo: Ubuntu 22.04 LTS

Distribución ROS: ROS 2 Humble Hawksbill


##Instalación y Configuración
Siga estos pasos para configurar el entorno de desarrollo local:

Clonar el repositorio:
Cree un espacio de trabajo y clone el código fuente en la carpeta src:

  mkdir -p ~/naranjelio_ws/src
  cd ~/naranjelio_ws/src
  git clone https://github.com/Proyecto-Cuadrupedo/Naranjelio_ws.git 

Desde la raíz del workspace (~/naranjelio_ws), ejecute:
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y

Para compilarlo
  colcon build --symlink-install
  source install/setup.bash


##Arquitectura
###Nodos
motor_controller: /naranjelio_control,Interfaz de bajo nivel con los servomotores y controladores de hardware.
kinematics_node: /naranjelio_kinematics,Procesamiento de cinemática inversa y directa para la locomoción.
teleop_node: /naranjelio_teleop,Interpretación de comandos de entrada (Teclado/Joystick).
state_publisher: /naranjelio_description,Publicación de estados de las articulaciones basado en el URDF.

###Interfaces (Servicios y Tópicos)
El sistema utiliza las siguientes interfaces principales para la comunicación entre procesos:

Tópicos Suscritos:
/cmd_vel (geometry_msgs/msg/Twist): Comandos de velocidad para el movimiento general.
/joint_group_position_controller/commands (std_msgs/msg/Float64MultiArray): Posiciones directas de los motores.

Tópicos Publicados:
/joint_states (sensor_msgs/msg/JointState): Estado actual de cada articulación.
/robot_description (std_msgs/msg/String): Modelo cinemático del robot.

Servicios:
/calibrate_motors: Inicia la rutina de calibración de posición cero.
/set_gait_mode: Cambia el perfil de marcha (trote, caminata, etc.).




