import socket
import time
import psutil as PS
import socket
import board
import digitalio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import threading as tr

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


# 메시지 대기
while True:
    data = conn.recv(1024)
    print("받은 메시지에 대한 응답을 전송했습니다: " + data.decode("utf-8"))

    # 메시지 처리
    if data.decode("utf-8") == "sleep":
        print("sleep receive")
        OLED_ON()


# 연결 닫기
conn.close()
p1.join()
