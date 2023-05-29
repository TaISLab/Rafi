Para controlar la plataforma usando el mando seguir los siguientes pasos:

- Concectar las dos controladores ROBOCLAW por usb al ordenador. Deben aparecer
  conectadas las direccione /dev/ttyACM0 y /dev/ttyACM1
- Conectar el mando por bluethooh al ordenador y mirar como ha sido referido,
  veremos algo del tipo jsX.
- En el archivo "roboclaw_2control.launch" cambiar en los parámetros del
  'joy node' el nombre de referencia del mando por el que se haya visto:
  "/dev/input/jsX"
- Ya se puede ejecutar el archivo "roboclaw_2control.launch" y controlar la
  plataforma. 

Nota: Si da error al ejecutar, en los argumentos iniciales del .launch cambiar
'dev' por 'dev2' o más sencillo desconectar los usb de las controladores y
volver a conectarlos en orden contrario.
