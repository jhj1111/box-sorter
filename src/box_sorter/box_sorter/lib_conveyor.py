import serial
import time
import json
import threading

########################connect arduino#################################################
# 아두이노가 연결된 포트 설정 (예: COM3 또는 /dev/ttyUSB0)
arduino_port = "/dev/ttyACM0"  # 리눅스: "/dev/ttyUSB0", 윈도우: "COM3"
baud_rate = 115200  # 아두이노 코드에서 설정한 속도

try:
    # 시리얼 포트 열기
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # 아두이노가 시리얼 통신을 초기화할 시간을 줌
    print("아두이노와 연결되었습니다.")
    #ser = True

except serial.SerialException:
    print(f"오류: {arduino_port} 포트에 연결할 수 없습니다. 포트가 올바른지 확인하세요.")
    ser = None  # 프로그램 종료 시 안전한 처리를 위해 변수 초기화

except FileNotFoundError:
    print(f"오류: {arduino_port} 포트를 찾을 수 없습니다. 아두이노가 연결되었는지 확인하세요.")
    ser = None

except OSError as e:
    print(f"시스템 오류 발생: {e}")
    ser = None

except serial.SerialTimeoutException:
    print("오류: 아두이노 응답 없음. 연결을 다시 확인하세요.")
    ser = None
#########################################################################################

# 현재 상태 저장 변수 (초기값은 None)
current_status = None
running = True  # 쓰레드 실행 상태

def read_status():
    """쓰레드에서 실행될 상태 읽기 함수"""
    global current_status, running
    
    while running:
        if ser.in_waiting > 0:  # 읽을 데이터가 있으면
            received_data = ser.read(1).decode().strip()  # 한 글자만 읽기
            if received_data:
                if received_data == '.' and current_status != "READY":
                    print("'conveyor/status': READY")
                    current_status = "READY"
                elif received_data == '_' and current_status != "RUN":
                    print("'conveyor/status': RUN")
                    current_status = "RUN"

        #time.sleep(0.1)  # CPU 점유율 방지
        return current_status
    return

def send_command(command, distance=None):
    """JSON 형식으로 명령어를 아두이노에 전송"""
    data = {"control": command}
    
    # stop 명령일 경우 distance.mm = 1 설정
    if command == "stop":
        data["distance.mm"] = 1  
    elif distance is not None:
        data["distance.mm"] = distance  # 'go' 명령어일 경우 거리 추가

    # if command =='stop' or command == 'go' :
    #     distance = distance if command == 'go' else 1
    #     ser.write(f"{distance}\n".encode())
    #     print(f"Sent: {distance}mm")
    # else :
    json_data = json.dumps(data)  # JSON 문자열 변환
    ser.write(f"{json_data}\n".encode())  # 아두이노로 전송
    print(f"Sent: {json_data}")

def disconnect_arduino():
    global running, status_thread
    if ser:
        running = False
        if status_thread and status_thread.is_alive():
            status_thread.join()  # 쓰레드가 종료될 때까지 대기
        ser.close()
        print("아두이노 연결이 종료되었습니다.")

# 상태 읽기 쓰레드 시작
if ser:
    status_thread = threading.Thread(target=read_status, daemon=True)
    status_thread.start()
else :
    running = False


#ser.close()  # 프로그램 종료 시 포트 닫기


