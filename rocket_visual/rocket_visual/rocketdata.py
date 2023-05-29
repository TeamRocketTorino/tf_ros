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

    def from_array(self, array):
        self.timestamp = int(array[0])

        self.ax = float(array[1])
        self.ay = float(array[2])
        self.az = float(array[3])
        self.gx = float(array[4])
        self.gy = float(array[5])
        self.gz = float(array[6])

        self.q1 = float(array[7])
        self.q2 = float(array[8])
        self.q3 = float(array[9])
        self.q0 = math.sqrt(1.0 - (self.q1**2 + self.q2**2 + self.q3**2))

        self.pressure = float(array[10])
        self.altitude = float(array[11])
        self.temperature = float(array[12])

        return self

    def from_string(self, str):
        if(len(str) == 0):
            return None
        
        str = str.strip()
        values = str.split(",")

        if(len(values) <= 13):
            return None
        
        values = values[:13]

        return self.from_array(values)
