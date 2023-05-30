import math
from dataclasses import dataclass
from geometry_msgs.msg import TransformStamped

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

    def compute_q0(self):
        partial = 1.0 - (self.q1**2 + self.q2**2 + self.q3**2)
        if partial < 0:
            self.q0 = math.nan
        else:
            self.q0 = math.sqrt(partial)

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
        self.compute_q0()

        self.pressure = float(array[10])
        self.altitude = float(array[11])
        self.temperature = float(array[12])

        return self

    def from_string(self, str):
        if(len(str) == 0):
            return None
        
        str = str.strip()
        values = str.split(",")

        if(len(values) < 13):
            return None
        
        values = values[:13]

        return self.from_array(values)
    
    def to_tf2(self, header_frame_id, child_frame_id):
        if math.isnan(self.q0):
            return None

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp.sec = int(self.timestamp/1000000)
        t.header.stamp.nanosec = int(self.timestamp%1000000)*1000
        t.header.frame_id = header_frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.altitude

        t.transform.rotation.x = self.q0
        t.transform.rotation.y = self.q1
        t.transform.rotation.z = self.q2
        t.transform.rotation.w = self.q3

        return t
