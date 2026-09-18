from dynamixel_sdk import *
PORT='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAA0974-if00-port0'  # 네 U2D2 경로
BAUD=57600
LEFT, RIGHT = 8, 9  # 네 세팅

port=PortHandler(PORT); pkt=PacketHandler(2.0)
assert port.openPort(), 'open fail'
assert port.setBaudRate(BAUD), 'baud fail'

def w1(i,a,v): print('w1',i,a,v, pkt.write1ByteTxRx(port,i,a,int(v)))
def w4(i,a,v): print('w4',i,a,v, pkt.write4ByteTxRx(port,i,a,int(v)))

# 속도모드(1) + 토크 ON
for i in (LEFT, RIGHT):
    w1(i,64,0); w1(i,11,1); w1(i,64,1)

# 구동 (좌 +200 LSB, 우 -200 LSB)
w4(LEFT,104,200); w4(RIGHT,104,-200)
input('ENTER to stop')

# 정지 + 토크OFF
w4(LEFT,104,0); w4(RIGHT,104,0)
w1(LEFT,64,0); w1(RIGHT,64,0)
port.closePort()

