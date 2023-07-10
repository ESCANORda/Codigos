import RPi.GPIO as GPIO
import smbus 
from time import sleep
import datetime
import pytz
import pysolar.solar as solar
import random

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Set the servo motor pins as output pins
GPIO.setup(4, GPIO.OUT)  # Servo motor 1
GPIO.setup(17, GPIO.OUT)  # Servo motor 2

pwm1 = GPIO.PWM(4, 50)
pwm1.start(0)
pwm2 = GPIO.PWM(17, 50)
pwm2.start(0)

# some MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

def angle_servo1(angle):
    duty = angle / 18 + 2
    GPIO.output(4, True)
    pwm1.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(4, False)
    pwm1.ChangeDutyCycle(0)

def angle_servo2(angle):
    duty = angle / 18 + 2
    GPIO.output(17, True)
    pwm2.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(17, False)
    pwm2.ChangeDutyCycle(0)

def set_position(azimuth, elevation):
    angle_servo1(azimuth)
    angle_servo2(elevation)

def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    
    # Concatenate higher and lower value
    value = ((high << 8) | low)
        
    # To get signed value from MPU6050
    if value > 32768:
        value = value - 65536
    return value

def avaliar_fitness(azimute, elevacao):
    # Obter a data e hora atual
    agora = datetime.datetime.now(pytz.utc)

    # Calcular a posição do sol
    altitude_graus = solar.get_altitude(latitude_graus, longitude_graus, agora)
    azimute_graus = solar.get_azimuth(latitude_graus, longitude_graus, agora)

    # Calcular o ângulo de diferença entre a posição do sol e a posição desejada pelo seguidor solar
    dif_azimute = abs(azimute_graus - azimute)
    dif_elevacao = abs(altitude_graus - elevacao)

    # Quanto menor a diferença, melhor é o desempenho do seguidor solar
    fitness = 1 / (1 + dif_azimute + dif_elevacao)
    return fitness

def mutacao(individuo):
    azimute, elevacao = individuo
    azimute_mutado = random.uniform(azimute_minimo, azimute_maximo)
    elevacao_mutada = random.uniform(elevacao_minima, elevacao_maxima)
    return azimute_mutado, elevacao_mutada

def cruzamento(pai1, pai2):
    azimute1, elevacao1 = pai1
    azimute2, elevacao2 = pai2
    ponto_corte = random.randint(0, 1)  # 0 para azimute, 1 para elevacao
    if ponto_corte == 0:
        filho = (azimute1, elevacao2)
    else:
        filho = (azimute2, elevacao1)
    return filho

def algoritmo_genetico():
    # Inicialização da população
    populacao = []
    for _ in range(tamanho_populacao):
        azimute = random.uniform(azimute_minimo, azimute_maximo)
        elevacao = random.uniform(elevacao_minima, elevacao_maxima)
        individuo = (azimute, elevacao)
        populacao.append(individuo)

    for geracao in range(num_geracoes):
        # Avaliação da população
        escores_fitness = []
        for individuo in populacao:
            fitness = avaliar_fitness(individuo[0], individuo[1])
            escores_fitness.append((individuo, fitness))

        # Ordenar por fitness em ordem decrescente
        escores_fitness.sort(key=lambda x: x[1], reverse=True)

        # Seleção dos pais (50% melhores indivíduos)
        pais = [individuo for individuo, _ in escores_fitness[:tamanho_populacao // 2]]

        # Geração da próxima geração
        proxima_geracao = pais[:]

        # Cruzamento e mutação
        while len(proxima_geracao) < tamanho_populacao:
            pai1 = random.choice(pais)
            pai2 = random.choice(pais)
            filho = cruzamento(pai1, pai2)

            # Mutação
            if random.random() < taxa_mutacao:
                filho = mutacao(filho)

            proxima_geracao.append(filho)

        populacao = proxima_geracao

    # Retornar o melhor indivíduo da última geração
    melhor_individuo = escores_fitness[0][0]
    return melhor_individuo

# Parâmetros do algoritmo genético
tamanho_populacao = 50  # Tamanho da população
num_geracoes = 100  # Número de gerações
taxa_mutacao = 0.1  # Taxa de mutação

# Parâmetros do seguidor solar
azimute_minimo = 0  # Valor mínimo do ânguloPeço desculpas, houve uma interrupção acidental na minha resposta anterior. Segue o restante do código:

```python
azimute_maximo = 180  # Valor máximo do ângulo de azimute
elevacao_minima = 0  # Valor mínimo do ângulo de elevação
elevacao_maxima = 180  # Valor máximo do ângulo de elevação

# Configurações do local do seguidor solar
latitude_graus = -13.2477  # Latitude do local (exemplo: Bom Jesus da Lapa-BA)
longitude_graus = -43.4148  # Longitude do local (exemplo: Bom Jesus da Lapa-BA)

# Pega a posição gerada pelo algoritmo genético
melhor_solucao = algoritmo_genetico()
melhor_azimute, melhor_elevacao = melhor_solucao

# Inicialização do sensor MPU6050
MPU_Init()

# Loop de controle
while True:
    # Leitura dos dados do sensor MPU6050
    acc_x = read_raw_data(ACCEL_XOUT)
    acc_y = read_raw_data(ACCEL_YOUT)
    acc_z = read_raw_data(ACCEL_ZOUT)

    # Cálculo dos valores de aceleração
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0 
    Az = acc_z / 16384.0

    # Mapeamento dos valores de aceleração para os intervalos de ângulo desejados para azimute e elevação
    in_min = 1
    in_max = -1
    out_min = 0
    out_max = 180
    azimuth = (Ay - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    elevation = (Ax - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # Verificação de posição correta
    if abs(azimuth - melhor_azimute) <= tolerancia and abs(elevation - melhor_elevacao) <= tolerancia:
        set_position(melhor_azimute, melhor_elevacao)
        print("Servo motores posicionados corretamente.")
    else:
        print("Posição incorreta. Realinhando os servomotores.")
        set_position(azimuth, elevation)
    
    # Aguarda um intervalo de tempo antes da próxima leitura
    sleep(0.08)
