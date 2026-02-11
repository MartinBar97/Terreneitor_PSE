# Funcionamiento y conexión con Terreneitor 
Para lograr una correcta conexión con el robot y poder ejecutar el código del siguelineas, es necesario realizar el siguiente procedimiento:
## 1 Conexión con el robot
Para poder realizar la comunicación con la Raspberry Pi del robot, es necesario que este esté conectado a la misma red que el equipo que se utilizará para realizar el proyecto. Para lograr este objetivo, primero se conetará mediante un cable RJ45 la Rasberry Pi directamente al Router de la red.
Tras esto, en el ordenador mediante la aplicación del Router, se buscará entre todos los dispositivos conectados el que tenga de nombre ROVER2 y se copiará la IP del mismo. Gracias a esta IP, se realizará un comando SSH en el terminal de Ubuntu desde el ordenador para iniciar una comunicación con el robot.
Para finalizar, una vez iniciada la comunicación, se configurará la Rasberry Pi con el Host de la red y la contraseña con el fin de que el robot pueda conectarse automáticamente al Wi-Fi. Para verificar que el proceso se ha realizado de manera correcta, se debe desconectar el robot del Router y realizarle un reinicio.

Tras ello, se debe verificar que este automáticamente se conecta a la red Wi-Fi a traves de la aplicación del mismo. En el caso de que no se encuentre el dispositivo ROVER2, será necesario repetir los pasos del proceso descrito.
## 2 Configuración inicial del robot
Tras haber realizado la conexión con el robot, se deben instalar los archivos de configuración en la Rasberry Pi. Para ello se ejecutará en el terminal de Ubuntu el siguiente comando:
  ```bash
  git clone https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi.git
  ```
Estos arcivos contienen toda la información base que necesita el robot para poder actuar los distitos elementos con los que cuenta (sensores, camara, motores...). Tras acabar esta instalación, se creará una nueva carpeta dentro del sistema de la Rasberry Pi que será en la que se colocarán los archivos del proyecto.
Es de vital importancia incluir dentro de la carpeta que se acaba de generar el archivo Server copiandolo desde dentro de la carpeta de Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi que se clonó en el paso anterior. Esta permitirá enlazar los códigos del proyecto con los drivers de control de los elementos del robot.
