

# Práctica 3. Programación de tareas en robots móviles
**Robots Móviles. Universidad de Alicante. Noviembre 2023**

En esta práctica vamos a programar un robot móvil para que realice una tarea estructurada, que implique coordinar una serie de tareas individuales. El objetivo es que sirva para ver conceptos y técnicas que os puedan ser útiles para desarrollar el proyecto de la asignatura durante el último mes de clase.

## Tarea a desarrollar

La tarea que debe realizar el robot en esta práctica es muy sencilla y es la siguiente: 

1. Inicialmente debe moverse al azar evitando obstáculos hasta que detecte un objeto de color rojo en su campo de visión
2. Una vez detectado, el robot debe imprimir un mensaje en la consola y debe volver al punto del que partió inicialmente. 

En general para desarrollar cualquier tarea necesitaréis saber cómo implementar distintos elementos:

- Subtareas de **movimiento**:
    + Algunas subtareas deben mandar comandos de movimiento al robot (por ejemplo: "navega en línea recta", "navega al azar evitando obstáculos".
    + Si tienes un mapa del entorno, algunas subtareas pueden requerir ***navegar* a puntos concretos del mapa** (por ejemplo una tarea de vigilancia). Para eso podéis usar el *stack* de navegación de ROS que ya usásteis en la práctica anterior. En esta se dan algunas pistas  de cómo usarlo desde código Python.   
- Otras subtareas serán de **detección de condiciones** (por ejemplo, "detectar si estoy en un pasillo", o "buscar en la imagen de la cámara una pelota de color rojo"). Para estas tendréis que hacer uso de los sensores del robot.
- Necesitaréis **coordinar las subtareas**: por ejemplo habrá subtareas que se deberán realizar en una secuencia ("primero ve al *waypoint* 1 y luego al 2"), otras serán condicionales ("navega aleatoriamente hasta que te encuentres una pelota"), otras tareas serán en paralelo... En robótica para coordinar este tipo de subtareas se pueden usar varios formalismos, como las máquinas de estados finitos y los *behavior trees*.

## El entorno para las pruebas

si tenéis ya creado un *workspace* de ROS podéis usarlo, en caso contrario tendréis que crear uno:

```bash
mkdir rob_mov_ws
cd rob_mov_ws
mkdir src
cd src
```

puedes copiar el repositorio actual como un *package* dentro de la carpeta `src` del workspace creado:

```bash
git clone https://github.com/ottocol/navigation_stage.git
```

y ahora compila el workspace y actualiza las variables de entorno para incluirlo

```bash
cd .. #ahora deberías estar en el directorio base del workspace (rob_mov_ws)
catkin_make
source devel/setup.bash
```

Para poner en marcha el simulador *stage* junto con `rviz` y el *stack* de navegación:

```bash
roslaunch navigation_stage mi_navigation.launch
```

Verás el mundo en el simulador *stage* y el mapa que ya ha sido construido por el robot en  `rviz`. Como el *stack* de navegación está en marcha deberías poder llevar al robot a cualquier punto con el botón `2D Nav Goal`.

Los sensores simulados del robot son un laser y una cámara. Para ver la imagen de la cámara puedes usar la aplicación `rqt_image_view`. El *topic* de la cámara simulada es `/image`.

La carpeta `src` del *package* contiene los ejemplos que se describirán a continuación y que te pueden servir de base para implementar la práctica.

## Procesar las imágenes de la cámara

Aunque no todas las tareas en robótica móvil requieren vision artificial, es un sensor que puede proporcionar información muy útil. Los turtlebot tienen una cámara [Orbbec Astra](https://www.roscomponents.com/es/camaras/orbbec) que nos puede proporcionar información 2D y 3D. Aquí solo veremos cómo procesar la imagen 2D. 

OpenCV está integrado con ROS y es la biblioteca que se usa normalmente para procesamiento de imágenes.

En [este ejemplo](https://github.com/ottocol/navigation_stage/blob/main/src/color_detector.py) podéis ver el código Python de un nodo de ROS que detecta y cuenta los píxeles de color rojo en la imagen captada por la cámara. Si el número de pixeles supera un umbral, publica este número en un determinado *topic*. 


## Coordinar tareas con máquinas de estados

Para programar un robot que lleve a cabo una tarea compleja resulta útil dividirla en subtareas que habrá que organizar y coordinar. Aunque la coordinación se puede llevar a cabo simplemente con las estructuras de control del lenguaje de programación que estemos usando (bucles, condicionales,...), en general será útil tener algún formalismo que nos permita representar y coordinar tareas. Uno de los más usados en robótica es el de las máquinas de estados.

Uno de los paquetes más sencillos y a la vez más flexibles para programar máquinas de estados con Python en ROS es [SMACH](http://wiki.ros.org/smach).

Si no tienes SMACH instalado puedes hacerlo con estos comandos (sustituye "noetic" por tu distribución de ROS si no es esa, por ejemplo "kinetic" o "melodic"):

```bash
sudo apt-get install ros-noetic-smach ros-noetic-smach-ros ros-noetic-
   executive-smach ros-noetic-smach-viewer
```

En [este ejemplo](https://github.com/ottocol/navigation_stage/blob/main/src/test_smach.py) puedes ver una máquina de estados sencilla con solo 2 estados, que van cambiando de uno a otro tras 1 segundo cada uno de ellos. Cuando el estado "DOS" se ha ejecutado ya 3 veces devuelve un valor que hace que la máquina acabe. Si lo ejecutas verás en la consola cómo va cambiando de estado. Si quieres verlo de forma gráfica puedes usar el comando `rosrun smach_viewer smach_viewer.py` (este comando solo funciona si en el código de tu programa inicializas y ejecutas el "SMACH Introspection Server", en el ejemplo es la variable `sis`).

SMACH tiene diversos tipos de estados predefinidos que nos permiten realizar ciertas tareas en ROS de manera sencilla sin tener que escribir demasiado código. Por ejemplo si un estado lo único que necesita hacer es llamar a un `Action` de ROS podemos usar un `SimpleActionState`, que solo necesita que le digamos el tipo de la acción y sus parámetros. Como la acción `MoveBaseAction` nos permite movernos a un punto del mapa planificando la mejor trayectoria y evitando obstáculos usar este tipo de estado es la mejor forma si queremos que en un estado el robot se mueva a un determinado punto.

[Este ejemplo](https://github.com/ottocol/navigation_stage/blob/main/src/smach_actionstate.py) ilustra cómo llamar a un `MoveBaseAction` de ROS con  un estado de SMACH de tipo `SimpleActionState`. Se definen dos estados correspondientes a dos acciones de movimiento a distintos puntos del mapa y la máquina de estados va alternando entre los dos de manera indefinida (en este caso no termina, tendréis que pararlo con Ctrl-C). 

## Ayuda para la implementación

> Lo que se pone a continuación es solo una sugerencia para el algoritmo de evitación de obstáculos pero podéis implementarlo como queráis. No se exigirá un algoritmo que funcione en todos los casos y "no choque nunca" ya que no es una tarea tan sencilla como parece.

Para implementar el algoritmo de movimiento al azar evitando obstáculos de manera simple podéis tomar dos "rayos" del laser que apunten uno hacia adelante a la izquierda y otro adelante a la derecha y moverse en función de estos:

- El comando de movimiento es de tipo `Twist`
- La velocidad de giro (campo `angular.z` del `Twist`) debería ser proporcional a la diferencia de distancias detectada por los dos rayos, y siempre hacia la dirección donde el obstáculo detectado está más lejos
- La velocidad lineal (campo `linear.x` del `Twist`) debe ser proporcional a la media de distancia del "rayo" derecho e izquierdo. De ese modo cuanto más lejos estén los obstáculos más rápido irá el robot
- Si la distancia media es inferior a un umbral podéis poner una velocidad de giro alta como "giro de emergencia" porque el robot va a chocar.

### Baremo y plazo de entrega



- Hasta 6 puntos: implementar correctamente la funcionalidad pedida en el apartado "Tarea a desarrollar"
- Hasta 1 punto: el código fuente debe estar adecuadamente documentado con comentarios.
- Hasta 1 punto: realizar y documentar pruebas de la tarea, indicando si funciona siempre, falla en algún momento, posibles problemas o causas que creéis que tienen los fallos, etc. Documentadla a ser posible con videos que muestren el funcionamiento.
- Hasta 1 punto: Mejorar el nodo de detección de color rojo: en el código de ejemplo se está siempre intentando detectar el color rojo, cuando en la tarea de "vuelta a la base" ya no es necesario, por lo que estamos malgastando recursos computacionales. Modificar el código para que publicando un mensaje en un determinado *topic* podamos dejar de estar suscritos al *topic* `/image` y publicando otro mensaje distinto volvamos a suscribirnos.
- Hasta 1 punto: mejorar la tarea añadiendo algún paso adicional, por ejemplo una vez detectado el color rojo, moverse hacia el objeto rojo (manteniéndolo centrado en la imagen) hasta llegar a una distancia especificada de él.


La práctica se podrá entregar hasta el **lunes 27 de noviembre a las 23:59**


## Apéndice Moverse directamente a un punto del mapa en Python con `MoveBaseAction`


> **Con una máquina de estados de SMACH, en lugar de este código es mucho más sencillo usar un estado de tipo `SimpleActionState`**. No obstante lo ponemos aquí por si en el proyecto necesitáis realizar esta tarea fuera de una máquina de estados de SMACH.

En este ejemplo podéis ver cómo decirle al robot que navegue hasta un determinado punto del mapa. La ruta la calculará automáticamente el *stack* de navegación (siempre suponiendo que tenemos un mapa del entorno). El código ejecuta una **acción**, que es como se representan en ROS las tareas que tardan un tiempo en ejecutarse. En nuestro caso la acción es de tipo `MoveBaseAction`, las que se usan para mover al robot.

> En RViz, para saber las coordenadas de un punto, pulsad sobre el botón con un "+" de la barra de herramientas y en el cuadro de diálogo elegir `Publish Point`. Aparecerá una nueva herramienta con la que si pasáis el ratón por el mapa en RViz os irá diciendo las coordenadas en metros.

> Para probar el siguiente ejemplo no os hace falta ningún workspace de ROS si no queréis crearlo, simplemente podéis copiar el código a un archivo `test_movebase.py` y ejecutadlo con `python test_movebase.py <x_destino> <y_destino`.

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import sys

#Uso de la acción move_base en ROS para moverse a un punto determinado
#En ROS una acción es como una petición de un "cliente" a un "servidor"
#En este caso este código es el cliente y el servidor es ROS
#(en concreto el nodo de ROS 'move_base')
class ClienteMoveBase:
    def __init__(self):
        #creamos un cliente ROS para la acción, necesitamos el nombre del nodo 
        #y la clase Python que implementan la acción
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' esté activo`
        self.client.wait_for_server()

    def moveTo(self, x, y):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x   
        goal.target_pose.pose.position.y = y
        #La orientación es un quaternion. Tenemos que fijar alguno de sus componentes
        goal.target_pose.pose.orientation.w = 1.0

        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
        return self.client.get_result()

if __name__ == "__main__":
    if len(sys.argv) <= 2:
        print("Uso: " + sys.argv[0] + " x_objetivo y_objetivo")
        exit()      
    rospy.init_node('prueba_clientemovebase')
    cliente = ClienteMoveBase()
    result = cliente.moveTo(float(sys.argv[1]), float(sys.argv[2]))
    print(result)
    if result:
        rospy.loginfo("Goal conseguido!")
```

### Cancelar una `MoveBaseAction`

Se puede cancelar una acción llamando al método `cancel` del `SimpleActionClient`. Es muy habitual disparar la cancelación cuando se reciba un mensaje determinado en un topic propio. 

Tienes un ejemplo en [este archivo](movebase_stop.py). Mientras se ejecuta la acción, el código está escuchando mensajes en el topic `/comando`. La acción se cancelará si recibe el mensaje "STOP" en este topic. Si mientras el robot se está moviendo a su destino escribes en una terminal :

```bash
rostopic pub comando std_msgs/String STOP
```
El robot debería cancelar la acción en curso. 


