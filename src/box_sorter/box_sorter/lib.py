import json, os
from std_msgs.msg import String


def flatten(lst):
    single = False
    if not isinstance(lst[0], list):
        yield lst
        single = True
        #return lst
    
    if not single:
        for item in lst:
            if isinstance(item, list) and isinstance(item[0], list):
                yield from flatten(item)
            else:
                yield item


def create_json_msg_test(command, data) -> String:
    """
    command 리스트를 기반으로 data 리스트를 JSON 형식으로 변환하여 ROS 2 String 메시지로 반환.

    :param command: JSON의 키 구조를 정의하는 리스트 (예: ['objects', ['class_id', 'x', 'y']])
    :param data: JSON의 값을 포함하는 리스트 (예: [0, 0.05, -0.002])
    :return: std_msgs.msg.String 메시지 (JSON 변환된 데이터 포함)
    """
    # if len(command) < len(data) :
    #     print('invalid data')
    #     return
    json_msg = String()

    dict_ = []
    for i, c in enumerate(flatten(command[::-1])):
        header = True
        data = data if isinstance(data[0], list) else [data]
        for d in data:
            dict_.append(dict(zip(c, d)))
            header = False
        if header:
            dict_ = {c : [dict_]}

    json_msg.data = json.dumps(dict_)

    return json_msg

def create_json_msg(command, data) -> String:
    json_msg = String()

    json_msg.data = json.dumps(dict(zip(command, data)))

    return json_msg

def add_offset(file_path = 'offset_values.txt', yolo_x=0, yolo_y=0):
    right_low_x_offset, right_low_y_offset, right_high_x_offset, right_high_y_offset, \
    left_low_x_offset, left_low_y_offset, left_high_x_offset, left_high_y_offset = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            for line in file:
                # 각 줄을 "변수명 : 값" 형식으로 분리하여 변수에 값을 넣음
                parts = line.strip().split(":")
                if len(parts) == 2:
                    var_name, value = parts
                    try:
                        # 값을 float로 변환하여 변수에 할당
                        value = float(value.strip())
                        if var_name == "right_low_x_offset ":
                            right_low_x_offset = value
                        elif var_name == "right_low_y_offset ":
                            right_low_y_offset = value
                        elif var_name == "right_high_x_offset ":
                            right_high_x_offset = value
                        elif var_name == "right_high_y_offset ":
                            right_high_y_offset = value
                        elif var_name == "left_low_x_offset ":
                            left_low_x_offset = value
                        elif var_name == "left_low_y_offset ":
                            left_low_y_offset = value
                        elif var_name == "left_high_x_offset ":
                            left_high_x_offset = value
                        elif var_name == "left_high_y_offset ":
                            left_high_y_offset = value
                    except ValueError:
                        pass
    else:
        # 파일이 없으면 새로운 파일을 생성하고 값을 저장
        with open(file_path, "w") as file:
            file.write(f"right_low_x_offset : {right_low_x_offset}\n")
            file.write(f"right_low_y_offset : {right_low_y_offset}\n")
            file.write(f"right_high_x_offset : {right_high_x_offset}\n")
            file.write(f"right_high_y_offset : {right_high_y_offset}\n")
            file.write(f"left_low_x_offset : {left_low_x_offset}\n")
            file.write(f"left_low_y_offset : {left_low_y_offset}\n")
            file.write(f"left_high_x_offset : {left_high_x_offset}\n")
            file.write(f"left_high_y_offset : {left_high_y_offset}\n")
    ################################################################################################
    if yolo_x > 0 and yolo_y > 0:   # right low
        yolo_robot_y = yolo_x + right_low_y_offset       # hight  += 아래   -= 위
        yolo_robot_x = yolo_y + right_low_x_offset       # width  += 오른쪽 -= 왼쪽
        print(f'right low  :  yolo_robot_x [{yolo_robot_x}] yolo_robot_y[{yolo_robot_y}]')
        # x : 0.03522528820800782] y : 0.0352 2528820800782]

    elif yolo_x > 0 and yolo_y < 0:  # right high
        yolo_robot_y = yolo_x + right_high_y_offset       # hight  += 아래   -= 위
        yolo_robot_x = yolo_y + right_high_x_offset       # width  += 오른쪽 -= 왼쪽
        #print("right high")
        print(f'right high  :  yolo_robot_x [{yolo_robot_x}] yolo_robot_y[{yolo_robot_y}]')

    elif yolo_x < 0 and yolo_y > 0:  # left low
        yolo_robot_y = yolo_x + left_low_y_offset       # hight  += 아래   -= 위
        yolo_robot_x = yolo_y + left_low_x_offset       # width  += 오른쪽 -= 왼쪽
        #print("left low")
        print(f'left low  :  yolo_robot_x [{yolo_robot_x}] yolo_robot_y[{yolo_robot_y}]')

    elif yolo_x < 0 and yolo_y < 0:  # left high
        yolo_robot_y = yolo_x + left_high_y_offset       # hight  += 아래   -= 위
        yolo_robot_x = yolo_y + left_high_x_offset       # width  += 오른쪽 -= 왼쪽
        #print("left high")
        print(f'left high  :  yolo_robot_x [{yolo_robot_x}] yolo_robot_y[{yolo_robot_y}]')

    return yolo_robot_x, yolo_robot_y

if __name__ == '__main__':
    #print(create_json_msg(['a', ['red', 'blue', 'goal']], [0, 1, 2]))
    command = ['objects', ['class_id', 'x', 'y', 'position']]
    data = [
        [0, 0.05, -0.002, 1],
        [1, -0.04, 0.0009, 2]
    ]
    print(create_json_msg_test(command, data))

    print(len(['a', ['red', 'blue', 'goal']]))
    l = []
    l2 = []
    for i in flatten([[7,8,9],[[1,2,3],[10,20,30]]]):
        l.append(i)
    for i in flatten([1,2,3]):
        print(f'f2 = {i}')
    print(l)
    print(l2)

    print(create_json_msg(['red', 'blue', 'goal'], [0, 1, 2]))