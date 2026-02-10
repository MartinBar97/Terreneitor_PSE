# Terreneitor - Siguelineas 

Este proyecto implementa un sistema de control para un robot sigue-líneas (**Freenove Smart Car**) utilizando Python.
La arquitectura está diseñada para separar la percepción de la acción, permitiendo un control suave y telemetría en tiempo real a través de una interfaz web.

## Características principales

-**Arquitectura Multihilo**: Separación total entre lectura de sensores, cálculo PID/Motores y comunicaciones para evitar retardos.
-**Control PID Dinámico**: Ajuste de constantes $K_p$, $K_i$ y $K_d$ en tiempo real desde el navegador.
-**Interfaz Web**: Visualización en vivo del estado de los tres sensores y errores mediante WebSockets.
-**Sistema de Pulso**: Control de tracción mediante micro-frenados(como si estuvieses "discretizado" o "parando y arrancando") para evitar que el robot salte a otras líneas,
    o se salga en curvas cerradas por exceso de inercia hacia curvas muy abiertas(tambien debido al circuito que teniamos de prueba).
-**Parada de Emergencia**: Doble sistema de seguridad de parada (Barra espaciadora + botón en Dashboard web).

---

## Lógica del Controlador PID

El corazón del Terreneitor es un controlador **PID (Proporcional, Integral y Derivativo)**.
Este algoritmo calcula la corrección necesaria para mantener al robot centrado en la línea basándose en la siguiente fórmula:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

### ¿Qué hace cada componente?
-**Proporcional ($K_p$)**: Reacciona al error actual. Si el robot está muy lejos de la línea, aplica un giro fuerte.
-**Integral ($K_i$)**: Corrige errores acumulados en el tiempo (útil si hay un desequilibrio constante en los motores).
-**Derivativo ($K_d$)**: Anticipa el futuro. Frena el giro cuando el robot se está acercando rápidamente al centro, evitando el zig-zag excesivo.

---

## Arquitectura del Sistema
El software se divide en componentes aislados que garantizan un control preciso:

1.  **Hilo de Sensores**: Realiza un muestreo de los infrarrojos a 1000Hz (cada 1ms).
2.  **Hilo de Motores**: Ejecuta el bucle de control PID y gestiona los actuadores con el sistema de pulsos.
3.  **Servidor WebSocket**: Transmite el estado del robot a 20Hz a los clientes conectados.
4.  **Servidor HTTP**: Aloja el Dashboard interactivo.

---

## Instalación y Uso

1.  Asegúrate de tener las librerías de Freenove en la carpeta `./Server`.
2.  Instala las dependencias necesarias:
    ```bash
    pip install websockets
    ```
3.  Ejecuta el programa:
    ```bash
    python SigueLineasV2.py
    ```
4.  Accede a la interfaz web desde cualquier dispositivo en la misma red. 
    * **Ejemplo**: `http://192.168.1.132:8080/`


---

## Notas de Desarrollo
El **Sistema de Pulso** (o de frenadas ya arrancadas, o como nosostros le llamamos, "discretizado") fue clave en el desarrollo, ya que durante las pruebas que hicimos,
el robot tendía a conservar demasiada energía cinética. Aplicando `PULSE_TIME = 0.004` cada pocos ciclos, logramos que el robot "muerda" mejor el circuito en las curvas complejas sin perder torque.
En los videos que subimos en uno se puede ver como en el circuito mas simple, con la linea negra tambien mas gorda, el robot iba bastante bien, pero con el mismo codigo en el otro, se salia muy facil de las curvas,
pero nos dimos cuenta que si haciamos microparadas, hacia mucho mejor las curvas, es por eso que en el codigo final dejamos ese codigo que hacia las microparadas, y como se aprecia en el video es capaz de seguir el circuito sin problema,
y como el ciclo de parada es rapido, se puede apreciar que hace el arranca para, pero es bastante continuo aunque no sea tanto como el otro.

