# Funcionamiento y conexión con Terreneitor 
Para lograr una correcta conexión con el robot y poder ejecutar el código del siguelíneas, es necesario realizar el siguiente procedimiento:

## 1. Conexión con el robot

Para poder establecer la comunicación con la Raspberry Pi del robot, es necesario que esté conectada a la misma red que el equipo desde el que se desarrollará el proyecto. Para ello, primero se conectará la Raspberry Pi directamente al router de la red mediante un cable RJ45.

A continuación, desde el ordenador y a través de la aplicación del router, se buscará entre los dispositivos conectados aquel que tenga el nombre **ROVER2**, y se copiará su dirección IP. Con esta IP, se ejecutará un comando SSH en el terminal de Ubuntu del ordenador para iniciar la comunicación con el robot.

Una vez iniciada la comunicación, se configurará la Raspberry Pi con el host de la red y la contraseña correspondiente, con el fin de que el robot pueda conectarse automáticamente al Wi-Fi. Para verificar que el proceso se ha realizado correctamente, se debe desconectar el robot del router y reiniciarlo.

Tras ello, se comprobará, mediante la aplicación del router, que el dispositivo se conecta automáticamente a la red Wi-Fi. En caso de que no aparezca el dispositivo **ROVER2**, será necesario repetir los pasos descritos anteriormente.

## 2. Configuración inicial del robot

Una vez realizada la conexión con el robot, se deben instalar los archivos de configuración en la Raspberry Pi. Para ello, se ejecutará en el terminal de Ubuntu el siguiente comando:
```bash
  git clone https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi.git
```

Estos archivos contienen toda la información base que necesita el robot para poder actuar sobre los distintos elementos con los que cuenta (sensores, cámara, motores, etc.). Tras completar esta instalación, se creará una nueva carpeta dentro del sistema de la Raspberry Pi, en la que se colocarán los archivos del proyecto.

Es de vital importancia incluir dentro de la carpeta recién creada el archivo `Server`, copiándolo desde la carpeta **Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi** clonada en el paso anterior. Este archivo permitirá enlazar los códigos del proyecto con los drivers de control de los distintos elementos del robot.

## 3. Lanzamiento y ejecución del código

Una vez copiados todos los archivos del proyecto en la carpeta generada, se procederá al lanzamiento y ejecución del código. Para ello, se abrirá un terminal de Ubuntu y se ejecutará un comando SSH para iniciar la comunicación con el robot.

Seguidamente, se ejecutará el comando `ls` para visualizar las carpetas disponibles y, a continuación, el comando `cd` para acceder a la carpeta del proyecto. Mediante una nueva ejecución del comando `ls`, se comprobarán los archivos del proyecto y, posteriormente, se creará con el comando `touch` un archivo vacío `.py` para la ejecución del código.

Una vez creado el archivo, se accederá a él mediante el comando `nano` y se pegará el código que se desea ejecutar utilizando `Ctrl + V`.

A continuación, se saldrá del editor con `Ctrl + X` y, al pulsar la tecla `Y`, se guardarán los cambios realizados en el archivo.

Finalmente, para ejecutar el código, se debe pulsar la tecla `Enter`, escribir el nombre del archivo `.py` creado en el paso anterior y volver a presionar `Enter`.
