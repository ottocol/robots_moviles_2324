# Sesión de trabajo con los Turtlebot: Mapeado, localización y navegación

En esta sesión de trabajo presencial con los Turtlebots vamos a probar los algoritmos de mapeado, localización y navegación ya implementados en ROS.

Los objetivos son:

- **Usar Rviz** para poder ver la información de los sensores del robot real
- **Probar el mapeado y localización** con el laser real
- **Probar la navegación autónoma** haciendo que el robot vaya de manera autónoma de un punto a otro sin chocar con obstáculos previstos e imprevistos.

## Conexión con el robot

> **IMPORTANTE**: en este modo de conexión que usaremos hoy **necesitáis ROS instalado en el PC. Para no tener problemas con la versión concreta de ROS instalada, lo mejor es que uséis los equipos de los laboratorios, ya que estas instrucciones asumen su uso. Usad los ordenadores 3,4,7,12 y 15, que tienen conexión wifi**. 

En el modo cliente/servidor, el cliente es nuestro PC y el servidor el robot. Hay algunos nodos de ROS que deben correr en el servidor (por ejemplo el `turtlebot_bringup.launch`) y otros en nuestro PC (por ejemplo `RViz`). En otros es indiferente.

> Es mejor ejecutar prácticamente todos los procesos en el robot salvo el rviz, os funcionará mejor en general.

Para empezar asegúrate que  **El PC se conecta con la red wifi del laboratorio**, recuerda que el nombre comienza por "labrobot". Usa a ser posible las que llevan un 5 en el nombre, son las de 5Ghz y deberían tener un mayor ancho de banda.

1. Descarga al PC el *script* [setvars.sh](setvars.sh) que fijará las variables de entorno necesarias:
    - Ejecútalo con `bash setvars.sh` seguido de un espacio y el número del robot con el que vas a trabajar, por ejemplo `bash setvars.sh 1`
    - Cierra la terminal y vuelve a abrirla. Si todo ha funcionado OK, debería aparecer un mensaje especificando tu IP y la del robot.

2. En una terminal **conecta con el robot**:

    > Para acortar, al ejecutar el archivo `setvars.sh` se habrá definido un comando `sshrobot` (sin argumentos) que actúa como alias de lo anterior, úsalo si quieres para no teclear tanto. El comando ya toma automáticamente la ip del robot, login y password.
    - Si no usas el comando `sshrobot`, recuerda que manualmente se haría con `ssh turtlebot@ip_del_robot` (en el robot 5 es `ssh tb2@ip_del_robot`), la contraseña es `ros`.
    - Recuerda que el turtlebot 1 es la `192.168.1.5` y así sucesivamente hasta el 5 que es la `192.168.1.9`.
    
    - En esta terminal arranca el robot: 
    
      ```bash
      roslaunch turtlebot_bringup minimal.launch
      ```
    - En otra **terminal del robot** arranca el laser (para abrir otra terminal en el robot necesitarás hacer `sshrobot`o `ssh turtlebot@ip_del_robot`  otra vez desde el PC)
     
      ```bash
      roslaunch turtlebot_bringup hokuyo_ust10lx.launch
      ```

4. Abre otra terminal de linux para **trabajar en tu PC**. 
    
5. Para comprobar que todo está OK, en la terminal abierta en el PC haz un `rostopic list`. Deberían aparecer los *topics* del ROS que está corriendo en el robot, entre ellos los de `/mobile_base`, el `/scan`, etc.

## Pruebas con RViz. Visualización de los sensores

Para visualizar los sensores del robot usamos RViz. En una **terminal en tu PC** ejecuta RViz con

```bash
rosrun rviz rviz
``` 

> Aparecerá un error de `fixed frame` en Global Options,  (el origen de coordenadas está puesto a `map`, pero de momento no hay un mapa). **Cambiar el `fixed frame` a `odom`**. Así usará como origen el punto (0,0,0) de la odometría.

### Visualizar el laser

Añadir una visualización para el laser (botón `Add` abajo a la izquierda > en el listado `By Display Type` seleccionar `LaserScan`). Una vez añadida aparecerá en el panel de la izquierda, cambiar el `topic` a `/scan`. Debería aparecer una línea con las distancias detectadas por el laser.

Para ver en RViz a dónde está mirando el robot puedes añadir sus ejes de coordenadas: botón `Add` abajo a la izquierda > en el listado `By Display Type` seleccionar `Axes`. Una vez añadido, en el panel de la izquierda desplegar el `Axes` y cambiar el Reference frame a `base_link`. El eje rojo es el X, que apunta hacia el frente, el Y el verde y el Z el Azul (siguiendo el orden clásico R-G-B). 

Puedes probar a mover el robot para ver cómo cambian las lecturas. **En una terminal en el robot** lanza el `roslaunch turtlebot_teleop keyboard_teleop.launch`. Recuerda que esta ventana tiene que tener el foco del teclado para que funcione.

> **Captura una pantalla en la que se vean las lecturas del laser (o haz una foto a la pantalla)** y luego con algún programa gráfico señala qué es lo que estaba "percibiendo" el robot en cada zona (poniendo un texto en cada zona que diga por ejemplo "pared", "mesa", "persona",...). Adjúntalo a la documentación del informe.

## Mapeado con teleoperación

En **una terminal en el robot**, lanza la teleoperación si no lo has hecho ya (`roslaunch turtlebot_teleop keyboard_teleop.launch`)

En otra **terminal del robot**, para crear el mapa escribe:

```bash
export TURTLEBOT_3D_SENSOR=astra
roslaunch turtlebot_navigation gmapping_demo.launch
```

En RViz añade un panel de tipo "Map" y cámbiale el topic a "/map" . Se debería ver el mapa conforme lo va construyendo el robot. 

Cuando lo tengas suficientemente completo, en una **terminal del robot** guárdalo con 

```bash
rosrun map_server map_saver -f nombre_que_quieras_dar_al_mapa
```

Recuerda que se crean dos ficheros, uno con extensión `.pgm`, que es el gráfico con el mapa en sí y otro con extensión `.yaml` que son los metadatos (tamaño en metros del total, tamaño en metros de cada pixel,...).

> Si quieres guardar una copia del mapa en el PC puedes ejecutar la misma instrucción en una terminal del PC, pero como mínimo deberías guardarlo en el robot, ya que luego le hará falta para la localización.


## Localización

Una vez guardado el mapa, interrumpe el gmapping_demo.launch que tenías funcionando para construir el mapa. Deja RViz funcionando para poder ver la localización.

En una **terminal conectada con el robot** haz: 

```bash
export TURTLEBOT_3D_SENSOR=astra
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/fichero_de_tu_mapa.yaml
```

donde: 

- sustituye "fichero_de_tu_mapa" por el nombre de tu mapa
- Fíjate en que tienes que poner la **trayectoria completa** desde la raíz del disco para que ROS lo localice. El directorio HOME del robot es `/home/turtlebot`

> **IMPORTANTE**: para visualizar correctamente lo que sigue, fijar en RViz como sistema de coordenadas `fixed frame` -> `map`

Para que el algoritmo de localización funcione **tienes que dar una estimación de la posición inicial en RViz**. 

1. Clica en el botón de la barra superior `2D pose estimate` 
2. Clica en el mapa, **en el punto en el que esté el robot en la realidad**. Una vez clicado, arrastra el ratón para indicar la dirección en la que mira el robot

Para visualizar las partículas del algoritmo de localización, añade en RViz un panel de tipo `PoseArray`. En el `topic` selecciona `/particlecloud`. Esto hará que aparezcan unas flechas rojas, que son las partículas con las posibles poses.

Las partículas se colocan por defecto al azar en un radio de aproximadamente un metro en torno a la posición estimada con el `2D pose estimate`. Para que las partículas se vayan "condensando" en una posición más precisa tendrás que teleoperarlo unos metros para que el algoritmo de localización actúe.

> Haz un par de fotos con tu móvil donde se vea la posición del robot en RViz y también la que tiene en el mundo real, para ver cómo se corresponden. Hazlo para 2-3 posiciones distintas, y adjúntalo a la documentación de la práctica.

## Navegación autónoma a un punto

Para que el robot **navegue automáticamente a un punto** le puedes fijar un destino en RViz con el botón de la barra superior llamado `2D Nav Goal`. Fíjate n que puedes clicar en el punto deseado y arrastrar para indicar la orientación final. IMPORTANTE: antes de esto, interrumpe el proceso de "teleop" ya que interferiría con la navegación.

Para ver en Rviz la trayectoria calculada por el algoritmo de planificación de trayectorias añade un panel de tipo `Path` y selecciona el *topic* `/move_base/NavfnROS/plan`

Fíjale un punto y comprueba si calcula una trayectoria razonable y se mueve correctamente hasta su destino (también puedes adjuntar un video del camino seguido aunque no es estrictamente necesario si has hecho el video anterior). El *stack* de navegación recalculará la trayectoria si en el camino encuentra obstáculos que no están en el mapa. Puedes probar a ponerte en medio de la trayectoria para ver si te detecta como obstáculo imprevisto y te evita en su camino.

## Entrega

La parte realizada hoy se adjuntará a la **memoria principal de la práctica 2** y debe incluir:

+ Nombres de las personas que habéis realizado la prueba
+ 1-2 páginas describiendo los resultados: si son buenos o malos en general, qué diferencias habéis notado con la simulación (si las hay). Incluid videos si podéis.
    + imágenes con la visualización del laser en Rviz
    + mapeado: poned una imagen del/los mapa/s, qué partes del mapa han salido mejor/peor, qué partes del laboratorio/mobiliario ha detectado mejor/peor el sensor... (y por qué puede ser, si se os ocurre), 
    +  localización: qué tal ha funcionado, si se pierde el robot en algún momento....
    +  navegación autónoma: qué tal ha funcionado, ¿llega el robot al destino? evita los obstáculos imprevistos?
+ Los archivos con el mapa generado durante la sesión  (.pgm y .yaml) (si hacéis más de una prueba, incluid todos los mapas generados).

Al ser el límite de la entrega de 20Mb, si no os cabe todo tendréis que guardar el/los video/s en Google Drive, o subirlo/s a Youtube o similar y luego poner los enlaces en la documentación.



    