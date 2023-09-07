# Instrucciones de instalación de ROS

Para las prácticas, en principio os vale cualquier versión de ROS 1, no os recomiendo usar ROS 2 si podéis evitarlo, ya que os complicaría bastante probar el código con los turtlebots del laboratorio, y los ejemplos que pondremos en clase siempre serán con ROS 1.

Tenéis varias opciones para usar ROS 1 en vuestro ordenador:

a) Usar una máquina virtual en cualquier SO
b) Instalar ROS en linux nativo
c) Instalar ROS en Windows

## Opción a) Usar una máquina virtual

Os dejamos una [imagen de una máquina virtual en VMWare](), con ROS Kinetic en formato `.ova` (para abrir el enlace anterior **debes identificarte con tu cuenta de gcloud de la UA**). 

En Windows puedes ejecutar la máquina virtual instalando el [VMWare Workstation Player](https://www.vmware.com/es/products/workstation-player/workstation-player-evaluation.html), que es gratuito, e importando la máquina con `File > Open...`.

> Aunque `.ova` es un formato también compatible con VirtualBox, "tradicionalmente" VirtualBox ha funcionado mal para emular ROS, no por ROS en sí sino por el simulador 3D Gazebo. Es posible que las versiones actuales de VirtualBox funcionen bien en este aspecto, pero no las he probado personalmente y no puedo asegurar nada.

Para comprobar que todo funciona OK, una vez arrancada la máquina virtual abre una terminal (la tienes en el menú de inicio, en la opción "Herramientas del Sistema > LXTerminal") y ejecuta la siguiente orden:

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Debería aparecer una ventana del simulador Gazebo con un mundo visto desde arriba con una pared hexagonal, unos obstáculos cilíndricos blancos y un robot Turtlebot 3 tipo "waffle".

## Opción b) Instalar ROS Noetic en linux nativo


> La ventaja que tiene esta forma de instalación es el rendimiento, sobre todo para Gazebo, que en algunos ordenadores puede tener muy bajo rendimiento en una máquina virtual.

Las versiones de ROS 1 están soportadas oficialmente en Ubuntu, y cada versión de ROS es para una versión concreta de Ubuntu. Se recomienda usar la última versión de ROS 1, Noetic, ya que es la que requiere de un Ubuntu más actual. Para ello necesitas tener **Ubuntu 20.04**. Las anteriores versiones de ROS requieren versiones aún más antiguas de Ubuntu, que es poco probable que a estas alturas quieras tener en tu ordenador de forma nativa.

Tendrás que seguir estos pasos:

### 1. Instalar ROS

Sigue todas las [instrucciones de instalación oficiales](http://wiki.ros.org/noetic/Installation/Ubuntu) para Noetic (en el paso 1.4 instala la versión `desktop-full`, que tiene todas las herramientas de ROS incluidas las gráficas). Tras terminar estos pasos tendrás ROS instalado pero te faltará algún simulador de robots móviles.

### 2. Instalar el simulador de Turtlebot 3

No tenemos estos robots en el laboratorio pero nos serán útiles para probar algunos algoritmos en simulación.

Necesitarás ejecutar las siguentes instrucciones en una terminal:

```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-simulations
```

Para comprobar que todo funciona, abre una nueva terminal y escribe:

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
```

Debería aparecer una ventana del simulador Gazebo con un mundo con paredes en forma de hexágono y unos obstáculos cilíndricos blancos, visto desde arriba.

### 3. Instalar el simulador de Turtlebot 2

Estos robots sí los tenemos en el laboratorio, de modo que lo que pruebes en el simulador luego podrás probarlo con los robots reales. Por desgracia, no tienen soporte oficial para Noetic, lo que nos va a obligar a compilar los fuentes.

Para descargar los fuentes de los diversos repositorios necesarios de manera automática necesitas bajarte primero este fichero [tb2.rosinstall](tb2.rosinstall). Bájatelo y déjalo en tu directorio $HOME (tu directorio personal, o sea `/home/tu_nombre_de_usuario`). 

En una terminal, ejecuta las siguientes instrucciones:

```bash
#dependencias de paquetes de Ubuntu
sudo apt install libusb-dev libftdi-dev python-is-python3 pyqt5-dev-tools

#dependencias de paquetes de Noetic
sudo apt install ros-noetic-openslam-gmapping ros-noetic-joy ros-noetic-base-local-planner ros-noetic-move-base

#directorio para los fuentes
mkdir $HOME/tb2_ws
cd $HOME/tb2_ws
#se baja los fuentes de los repos especificados en el .rosinstall
wstool init src ../tb2.rosinstall

#Ñapa para arreglar un problema de dependencias
rm -rf src/ar_track_alvar/ar_track_alvar

#Compilar.Tardará! :)
catkin_make_isolated
```

Tardará un rato en compilar todos los fuentes. Una vez terminada la compilación, para terminar la configuración hay que hacer que tengamos accesibles los paquetes de Turtlebot2 cada vez que abrimos una terminal:

```bash
echo "source $HOME/tb2_ws/devel_isolated/setup.bash" >> ~/.bashrc
```

Cierra la terminal y abre una nueva para que los cambios tengan efecto (también puedes hacer `source ~/.bashrc`).

Para comprobar que funciona, ejecuta en una terminal:

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```

Debería aparecer una ventana del simulador Gazebo con un robot Turtlebot2 rodeado de varios objetos. (La primera vez es normal que tarde ya que tiene que bajarse los modelos 3D de los objetos).

## Opción c): Instalar ROS en Windows


Seguid las [instrucciones oficiales de instalación](http://wiki.ros.org/Installation/Windows). Yo no lo he probado personalmente aunque debería funcionar...





