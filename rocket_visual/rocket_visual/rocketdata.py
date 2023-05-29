import math
from dataclasses import dataclass

@dataclass
class RocketData:
    timestamp: int = 0
    q0: float = 0.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    temperature: float = 0.0
    pressure: float = 0.0
    altitude: float = 0.0


def rocketdata_from_array(array):
    rd = RocketData()
    rd.timestamp = int(array[0])

    rd.ax = float(array[1])
    rd.ay = float(array[2])
    rd.az = float(array[3])
    rd.gx = float(array[4])
    rd.gy = float(array[5])
    rd.gz = float(array[6])

    rd.q1 = float(array[7])
    rd.q2 = float(array[8])
    rd.q3 = float(array[9])
    rd.q0 = math.sqrt(1.0 - (rd.q1**2 + rd.q2**2 + rd.q3**2))

    rd.pressure = float(array[10])
    rd.altitude = float(array[11])
    rd.temperature = float(array[12])
    return rd