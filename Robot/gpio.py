
import RPi.GPIO as GPIO


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


BUZZER = 10


ENA = 13
ENB = 20
IN1 = 16
IN2 = 19
IN3 = 26
IN4 = 21


ECHO = 4
TRIG = 17


IR_R = 18
IR_L = 27
IR_M = 22
IRF_R = 25
IRF_L = 1


GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)
ENA_pwm = GPIO.PWM(ENA, 1000)
ENA_pwm.start(0)
ENA_pwm.ChangeDutyCycle(100)
ENB_pwm = GPIO.PWM(ENB, 1000)
ENB_pwm.start(0)
ENB_pwm.ChangeDutyCycle(100)

GPIO.setup(IR_R, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(IR_L, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(IR_M, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(IRF_R, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(IRF_L, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setup(BUZZER, GPIO.OUT, initial=GPIO.LOW)


def digital_write(gpio, status):
	GPIO.output(gpio, status)
def digital_read(gpio):
	return GPIO.input(gpio)
def ena_pwm(pwm):

	ENA_pwm.ChangeDutyCycle(pwm)

def enb_pwm(pwm):
	ENB_pwm.ChangeDutyCycle(pwm)
