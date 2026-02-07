import time
from Server.motor import Ordinary_Car
from Server.infrared import Infrared

class LineFollower:
    def __init__(self):
        self.motor = Ordinary_Car()
        self.infrared = Infrared()
        self.speed_forward = 800          # velocidad recta
        self.speed_turn_medium = 1500     # giro suave
        self.speed_turn_sharp = 2000      # giro más brusco
        self.speed_turn_very_sharp = 4000 # giro muy brusco

    def follow_line(self):
        try:
            while True:
                value = self.infrared.read_all_infrared()
                print(f"Sensor IR: {value}")

                if value == 2:  # línea centrada
                    self.motor.set_motor_model(
                        self.speed_forward, self.speed_forward,
                        self.speed_forward, self.speed_forward
                    )
                elif value == 1:  # línea ligeramente izquierda
                    self.motor.set_motor_model(
                        self.speed_turn_very_sharp, self.speed_turn_very_sharp,
                        -self.speed_turn_medium, -self.speed_turn_medium
                    )
                elif value == 3:  # línea más a la izquierda
                    self.motor.set_motor_model(
                        self.speed_turn_very_sharp*2, self.speed_turn_very_sharp*2,
                        -self.speed_turn_sharp, -self.speed_turn_sharp
                    )
                elif value == 4:  # línea ligeramente derecha
                    self.motor.set_motor_model(
                        -self.speed_turn_medium, -self.speed_turn_medium,
                        self.speed_turn_very_sharp, self.speed_turn_very_sharp
                    )
                elif value == 6:  # línea más a la derecha
                    self.motor.set_motor_model(
                        -self.speed_turn_sharp, -self.speed_turn_sharp,
                        self.speed_turn_very_sharp*2, self.speed_turn_very_sharp*2
                    )
                elif value == 7:  # línea perdida / detener
                    self.motor.set_motor_model(0,0,0,0)
                else:
                    # cualquier valor inesperado → detener
                    print(f"Valor IR inesperado: {value}")
                    self.motor.set_motor_model(0,0,0,0)

                time.sleep(0.01)  # evita sobrecargar CPU

        except KeyboardInterrupt:
            print("Siguelíneas detenido")
            self.motor.set_motor_model(0,0,0,0)

if __name__ == "__main__":
    follower = LineFollower()
    print("Iniciando siguelíneas. Ctrl+C para detener.")
    follower.follow_line()

