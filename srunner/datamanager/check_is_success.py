import argparse
import datetime

import carla
from dateutil import parser


def _analyse(args):
    if args.testresultfile is None or args.testresultfile == '':
        print("please set test result file path using --testresultfile")
        return
    if args.recordfile is None or args.recordfile == '':
        print("please set recorded file path using --recordfile")

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)

        # 過濾test result file中記錄的由算法得出的car的碰撞結果
        test_result_all = {}
        with open(args.testresultfile, 'r', encoding='utf-8') as test_result_line:
            one_line_message = test_result_line.readline()
            while one_line_message:
                seperator_index = one_line_message.rfind(':')
                first_half = one_line_message[:seperator_index]
                second_half = one_line_message[seperator_index + 1:]

                # one_message_info_list = []  # 時間, car role_name, car id, algrithm detect result
                time_end_index = first_half.rfind(']')
                # one_message_info_list.append(first_half[1:time_end_index])  # algrithm detect time
                if second_half.startswith(' car'):
                    second_split = second_half.split(' ')
                    # 采用元组（id， role_name）作为key
                    if not test_result_all.__contains__((second_split[2], second_split[1])):
                        test_result_all[(second_split[2], second_split[1])] = []
                    test_result_all[(second_split[2], second_split[1])].append(first_half[1:time_end_index])  # 记录时间即可
                    # one_message_info_list.append(second_split[1])  # car role_name
                    # one_message_info_list.append(second_split[2])  # car id
                    # one_message_info_list.append(second_split[-1])  # algrithm detect result
                # else:
                #     one_message_info_list.append(second_half)

                # test_result_list.append(one_message_info_list)
                one_line_message = test_result_line.readline()

        # 場景中實際的碰撞結果
        collision_info_from_record = client.show_recorder_collisions(args.recordfile, 'v', 'v')
        split_collision_info = collision_info_from_record.split('\n')
        start_time_str = split_collision_info[2]
        start_time_str = start_time_str[6:]
        start_date_time = parser.parse(start_time_str)

        collision_info_all = []
        for i in range(5, len(split_collision_info) - 4):
            one_collision_str = split_collision_info[i]
            one_collision_str_split = one_collision_str.strip().split(' ')
            one_collision_info = []  # 碰撞時間，碰撞車的id，碰撞車的類型
            one_collision_info.append(start_date_time + datetime.timedelta(seconds=int(one_collision_str_split[0])))
            one_collision_info.append(one_collision_str_split[-2])
            one_collision_info.append(one_collision_str_split[-1])
            collision_info_all.append(one_collision_info)

        # 如果test result file 中的判斷碰撞時間早于 錄製文件中實際碰撞的時間，則認爲success， 反之 failed
        for item in collision_info_all:
            car_id = item[1]
            for k, v in test_result_all.items():
                if k[0] == car_id:
                    test_collision_time = v[0]
                    if (parser.parse(test_collision_time) - item[0]).days < 0:
                        print("%s 验证通过！" % (k[1]))
                    else:
                        print("%s 验证不通过！" % (k[1]))
                    break

    finally:
        pass


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        "--testresultfile",
        type=str,
        default='',
        help=' test result file path'
    )
    argparser.add_argument(
        "--recordfile",
        type=str,
        default='',
        help=' recorded file path'
    )
    args = argparser.parse_args()

    try:
        _analyse(args)
    except Exception as error:
        print(error)


if __name__ == '__main__':
    main()
