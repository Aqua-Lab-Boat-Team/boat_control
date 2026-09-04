from dataclasses import dataclass

@dataclass
class CommsConfig:

    USE_UDP:bool = True # UDP or UART -- Default to UDP
    UDP_PORT:str = "udpout:192.168.55.100:14550" # Jetson
    # UDP_PORT:str = "udpout:192.168.10.2:14550" # Raspberry pi

    RADIO_UART_PORT:str = "/dev/radio"
    RADIO_UART_BAUD:int = 115200

    GPS_UART_PORT:str = "/dev/gps"
    GPS_UART_BAUD:int = 38400

    ARDUINO_UART_PORT:str = "/dev/arduino"
    ARDUINO_UART_BAUD:int = 115200

    MVL_SYSID:int = 1
    MVL_COMPID:int = 1
    GCS_SYSID:int = 255
    HB_INTERVAL:float = 1.0          # send heartbeat every 1 second
    SYS_STAT_INTERVAL:float = 0.1    # send status every 0.1 second