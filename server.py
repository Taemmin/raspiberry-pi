import socket
import time
import psutil as PS
import socket
import board
import digitalio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import threading as tr
import serial
from gpiozero import Servo

servo = Servo(18)
ser = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

WIDTH = 128
HEIGHT = 64
FONTSIZE = 20

LOOPTIME = 1.0
# I2C 통신
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C)
# Clear display.
oled.fill(0)
oled.show()
# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new("1", (oled.width, oled.height))
# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)
# Draw a black filled box
draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
padding = 0
top = padding
bottom = oled.height - 40
x = 33
# font = ImageFont.load_default()
font1 = ImageFont.truetype("godoMaum.ttf", FONTSIZE)
font2 = ImageFont.truetype("godoMaum.ttf", FONTSIZE - 6)

# 서버의 IP 주소 또는 호스트 이름
HOST = "192.168.0.235"

# 서버에서 사용할 포트 번호 (클라이언트와 일치해야 함)
PORT = 12345

# 소켓 생성
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("소켓 생성됨")

# 오류 예외 처리 관리
try:
    s.bind((HOST, PORT))
except socket.error:
    print("바인드 실패")

# 클라이언트의 연결을 대기하는 소켓
s.listen(5)
print("메시지 대기 중인 소켓")
(conn, addr) = s.accept()
print("연결됨")

# OLED
global oled_state
oled_state = False


def OLED_ON():
    draw.text((x, top), "warning!!!", font=font1, fill=255)
    draw.text((x, bottom), "차량 운전자 \n졸음운전 중!!!", font=font2, fill=255)
    # Display image
    oled.image(image)
    oled.show()
    global oled_state
    if oled_state == False:
        print("thread 생성")
        p1 = tr.Thread(target=OLED_OFF)
        p1.start()


def OLED_OFF():
    global oled_state
    oled_state = True
    print("oled off시작")
    time.sleep(5)
    oled.fill(0)
    oled.show()
    oled_state = False
    print("oled off 끝")


# 진동모터
# def vibe():
#     try:
#         while 1:
#             motor_pwm.start(20)
#             print("pwm start")
#             time.sleep(3)
#             break
#     except:
#         print("error")
#     motor_pwm.stop()
#     GPIO.cleanup

# CO2 측정 및 창문 개폐
global window_open
window_open = False


def read_CO2():
    ser.write(b"\xFF\x01\x86\x00\x00\x00\x00\x00\x79")
    time.sleep(0.1)
    response = ser.read(9)
    if len(response) == 9 and response[0] == 0xFF and response[1] == 0x86:
        co2_level = (response[2] << 8) + response[3]
        return co2_level
    else:
        return None


# 창문 열림
def servo_open():
    for i in range(1, 4):
        servo.max()
        time.sleep(1)
    servo.value = None


# 창문 닫힘
def servo_close():
    for i in range(1, 4):
        servo.min()
        time.sleep(1)
    servo.value = None


def co2():
    global window_open
    while True:
        servo.value = None
        co2 = read_CO2()
        if co2 is not None:
            print("co2농도: ", co2)
            if co2 >= 2500 and window_open == False:
                servo_open()
                window_open = True
            elif co2 < 2500 and window_open == True:
                servo_close()
                window_open = False
        else:
            print("데이터를 읽는데 문제가 발생했습니다.")
        time.sleep(1)


p2 = tr.Thread(target=co2)
p2.start()
print("thread2 생성")
# 메시지 대기
while True:
    data = conn.recv(1024)
    print("받은 메시지에 대한 응답을 전송했습니다: " + data.decode("utf-8"))

    # 메시지 처리
    if data.decode("utf-8") == "sleep":
        print("sleep receive")
        OLED_ON()
        # vibe()

# 연결 닫기
conn.close()
