from dataclasses import dataclass

@dataclass
class CtrlCoeffs:
    # Distance controller
    kp_d: float = 0.1 # 0.1 starting -> 10m should produce full control effort
    ki_d: float = 0.00
    kd_d: float = 0.0

    # Heading controller
    kp_h: float = 0.005 # 0.02 starting -> 45 degrees should produce full control effort
    ki_h: float = 0.00
    kd_h: float = 0.0