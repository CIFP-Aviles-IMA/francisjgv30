#Docstring
"""
Este script controla un brazo robótico usando servomotores y un controlador PWM PCA9685  
conectado a una Jetson. El brazo incluye servos para las articulaciones (hombro, codo, muñeca, base y garra)
y potenciómetros que ajustan su posición en tiempo real. Un botón controla la apertura y cierre de la garra.

Requisitos: 
Jetson.GPIO: Control de pines GPIO.
adafruit_pca9685 y adafruit_servokit: Manejo del controlador PWM y servos.
time: Para retrasos en las acciones.

Funcionamiento:
Los servos reciben señales PWM del controlador PCA9685.
Los potenciómetros, conectados a GPIO, ajustan los ángulos de las articulaciones.
Un botón en GPIO 15 abre/cierra la garra.
Funciones principales:

moveMotor(controlIn, motorOut): Ajusta servos según valores de potenciómetros.
Bucle continuo que sincroniza servos con los potenciómetros y la garra con el botón.

Parámetros:
MIN_PULSE_WIDTH: 650 μs.
MAX_PULSE_WIDTH: 2350 μs.
FREQUENCY: 50 Hz.
 """

#import Wire
#import Adafruit_PWMServoDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
import time
i2c = busio.I2C(board.SCL,board.SDA)
from adafruit_servokit import Servokit


#variables globales
MIN_PULSE_WIDTH  =  650
MAX_PULSE_WIDTH  =  2350
FREQUENCY        =  50


#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
pwm = adafruit_pca9685.PCA9685("i2C")
kit = ServoKit(channels=16)


#Configuro el SetUP
time.sleep(5)                                   # Temporizador para la posicion incial del controlador 
pca.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = adafruit_motor.servo.Servo(0)           
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)

#varables de potenciometro
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)                     
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)                           #Configurar el gripper a 90 grados (Cerrar el gripper) X en Jetson.
pwm.begin()
GPIO.setup(29, GPIO.IN)                         # pin válido en jetson


def moveMotor(controlIn, motorOut):
  """ Descripción de las funciones DE: def MoveMotor(controlIN, motorOUT): 

    información:

   controlIn (int):Pin GPIO donde se lee el valor del potenciometro.
 motorOut (int):Pin de salida del motor el cual se utiliza para enviar la señal PWM al motor.
    
     
    Returns: En función del valor que tenga, se reiniciará la posición.
       
  """
  pulse_wide, pulse_width, potVal = -3
  
#  potVal = analogRead(controlIn);                                                   #Leer valor del Potentiometro
  potVal = GPIO.input(controlIN)
  pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);                #Mapear la posición del potenciómetro al motor"
  
#  pwm.setPWM(motorOut, 0, pulse_width);
  pwm = GPIO.PWM(motorOut, 0, pulse_width)
 
 
while (True):  
  moveMotor(potWrist, wrist)
  moveMotor(potElbow, elbow)                              #Asignar los motores a los potenciómetros correspondientes
  moveMotor(potShoulder, shoulder)
  moveMotor(potBase, base)
  pushButton = GPIO.input(29)      
  if(pushButton == GPIO.LOW):

    pwm.setPWM(hand, 0, 180);                            #Mantener el gripper cerrado cuando el botón no está presionado
    print("Grab")
  
  else:
  
    pwm.setPWM(hand, 0, 90);                              #Mantener el gripper abierto cuando el botón no está presionado
    print("Release")

GPIO.cleanup()
