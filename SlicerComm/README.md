# SlicerComm

SlicerComm is a 3D Slicer scripted module that creates configurable communication panels for interacting with external devices over **Serial**, **TCP/IP**, or **UDP**. 

## Features

- Serial: open/close port, set baud rate, send/read data.
- TCP/IP: client or server mode; connect, start server, accept clients, send/receive.
- UDP: bind or connect endpoints, send data with basic fragmentation.


## Layout
- `SlicerComm.py` — module entry point, widget, and logic.
- `Resources/UI/` — main module UI and protocol panel templates.
- `Scripts/Logics/`
  - `communication_serial.py`
  - `communicaton_TCPIP.py`
  - `communication_UDP.py`
  - `install_dependency.py`
- `Resources/Icons/SlicerComm.png` — toolbar icon.

## Usage
```python
logic = slicer.util.getModuleLogic("SlicerComm")

# 1) Serial
serial_comm = logic.createCommunication(
    communication_id=1,
    protocol="Serial",
    port="COM3",
    baudrate=115200,
)
serial_comm.setDelimiter("\n")
serial_comm.sendData("PING\n")
resp = serial_comm.readResponse()
serial_comm.closeSerialPort()

# 2) TCP/IP (client mode)
tcp_comm = logic.createCommunication(
    communication_id=2,
    protocol="TCP/IP",
    ip="192.168.1.50",
    port=5000,
)
tcp_comm.sendData(b"HELLO\n")
resp = tcp_comm.receiveData()
tcp_comm.disconnect()

# 3) UDP
udp_comm = logic.createCommunication(
    communication_id=3,
    protocol="UDP",
    ip="192.168.1.60",
    port=6000,
)
udp_comm.sendData("HELLO UDP", remote_ip="192.168.1.60", remote_port=6000)
data, addr = udp_comm.receiveData(timeout=1.0)
udp_comm.disconnect()

```

## Notes
- Dependencies are installed at module load via `install_dependency.py`.
- Panel templates live in `Resources/UI/Template_*CommunicationPanel.ui`.
- Communication helpers expose additional methods (e.g., `setDelimiter`, `startServer`, `bind`, `sendData`) for programmatic use.