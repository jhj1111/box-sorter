import json
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