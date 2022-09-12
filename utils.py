import io
from cyber_py import record
from datetime import datetime, timedelta
from google.protobuf.json_format import MessageToJson
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.drivers.proto.sensor_image_pb2 import Image, CompressedImage
from modules.routing.proto.routing_pb2 import RoutingRequest, RoutingResponse
from modules.transform.proto.transform_pb2 import TransformStampeds
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles
from modules.planning.proto.planning_pb2 import ADCTrajectory
import numpy as np
import time
from datetime import datetime

from scipy import spatial


def top_k(seg_order, seg_diff):
    top_k = None
    for i, seg in enumerate(seg_order):
        if not seg_diff[seg]:
            top_k = i + 1
            break

    if not top_k:
        top_k = -1

    return top_k


def apfd(seg_order, seg_diff):
    n = len(seg_order)
    # m = n - sum(seg_diff.values())
    bug_orders = []

    for i, seg in enumerate(seg_order):
        if not seg_diff[seg]:
            bug_orders.append(i + 1)

    m = len(bug_orders)

    # print(n, m, bug_orders, len(bug_orders))

    try:
        apfd = 1 - float(sum(bug_orders)) / (n * m) + 1.0 / (2 * n)
    except:
        apfd = -1

    return round(apfd, 2)

def check_direction(self_heading, obs_heading):
    direction = 0
    if abs(self_heading - obs_heading) <= 5:
        direction = 1
    elif abs(self_heading - obs_heading) >= 175 and abs(self_heading - obs_heading) <= 185:
        direction = 2
    elif abs(self_heading - obs_heading) >= 85 and abs(self_heading - obs_heading) <= 95:
        direction = 3

    return direction


def cal_similarity(a, b):
    if np.array_equal(a, b):
        similarity = 1
    else:
        # a = normalized(a)
        # b = normalized(b)
        # print(a, b)
        similarity = 1 - spatial.distance.cosine(a, b)
    return similarity


def get_majority(vec_list):
    unique_vecs = []
    for vec in vec_list:
        # check whether the vec is in the unique vec list
        flag = False
        for i in range(len(unique_vecs)):
            if unique_vecs[i][0] == vec:
                unique_vecs[i][1] += 1
                flag = True
                break

        if not flag:
            unique_vecs.append([vec, 1])

    # find the majority unique vec
    max_count = -1
    majority_vec = None
    for i in range(len(unique_vecs)):
        if unique_vecs[i][1] > max_count:
            max_count = unique_vecs[i][1]
            majority_vec = unique_vecs[i][0]

    # print(vec_list, majority_vec)
    return majority_vec



def vec_denoise(record_vec, window_size=3):
    new_record_vec = []

    for i in range(len(record_vec)):
        if i <= window_size//2:
            current_window = record_vec[0:window_size]
        elif i >= len(record_vec) - window_size//2:
            current_window = record_vec[-window_size:]
        else:
            current_window = record_vec[i-window_size//2:i+window_size//2+1]

        # print('current_window: ', current_window)
        major_vec = get_majority(current_window)
        # print('major_vec: ', major_vec)
        new_record_vec.append(major_vec)

    return new_record_vec

def vec_denoise_v2(record_vec, window_size=5):
    current_window = []
    new_record_vec = []
    for vec in record_vec:
        if len(current_window) < window_size:
            current_window.append(vec)
        else:
            print('current_window: ', current_window)
            major_vec = get_majority(current_window)
            new_record_vec += [major_vec] * window_size
            print(major_vec)
            current_window = [vec]

    new_record_vec.append(get_majority(current_window))
    new_record_vec = np.array(new_record_vec)

    return new_record_vec


def vec_denoise_v3(record_vec, window_size=5):
    new_record_vec = []
    for i in range(len(record_vec) - window_size):
        current_window = record_vec[i:i+window_size]
        major_vec = get_majority(current_window)
        # print(major_vec)
        new_record_vec.append(major_vec)

    # new_record_vec = np.array(new_record_vec)
    return new_record_vec


def get_alignment_messages(source_record, test_system=None):
    freader = record.RecordReader(source_record)
    localization_msgs = []
    light_msgs = []
    obstacle_msgs = []
    prediction_msgs = []
    planning_msgs = []

    localization_msg_count = 0
    light_msg_count = 0
    obstacle_msg_count = 0
    prediction_msg_count = 0
    planning_msg_count = 0


    for channelname, msg, datatype, timestamp in freader.read_messages():
        if channelname == '/apollo/perception/traffic_light':
            if localization_msg_count != 0:
                msg_new = TrafficLightDetection()
                msg_new.ParseFromString(msg)

            # print(channelname, timestamp)
                if localization_msg_count - light_msg_count == 1:
                    light_msgs.append(msg_new)
                    light_msg_count += 1
                elif light_msg_count == localization_msg_count:
                    light_msgs[-1] = msg_new

        elif channelname == '/apollo/planning':
            if localization_msg_count != 0:
                msg_new = ADCTrajectory()
                msg_new.ParseFromString(msg)
            # print(channelname, timestamp)
                if localization_msg_count - planning_msg_count == 1:
                    planning_msgs.append(msg_new)
                    planning_msg_count += 1
                elif planning_msg_count == localization_msg_count:
                    planning_msgs[-1] = msg_new

        elif channelname == '/apollo/prediction' and test_system != 'obstacle':
            if localization_msg_count != 0:
                msg_new = PredictionObstacles()
                msg_new.ParseFromString(msg)

                if localization_msg_count - prediction_msg_count == 1:
                    prediction_msgs.append(msg_new)
                    prediction_msg_count += 1
                elif prediction_msg_count == localization_msg_count:
                    prediction_msgs[-1] = msg_new

        elif channelname == '/apollo/perception/obstacles':
            if localization_msg_count != 0:
                msg_new = PerceptionObstacles()
                msg_new.ParseFromString(msg)

                if localization_msg_count - obstacle_msg_count == 1:
                    obstacle_msgs.append(msg_new)
                    obstacle_msg_count += 1
                elif obstacle_msg_count == localization_msg_count:
                    obstacle_msgs[-1] = msg_new


        elif channelname == '/apollo/localization/pose':
            msg_new = LocalizationEstimate()
            msg_new.ParseFromString(msg)
            if test_system != 'obstacle':
                if localization_msg_count == min(light_msg_count, obstacle_msg_count, planning_msg_count, prediction_msg_count):
                # if localization_msg_count == min(light_msg_count, obstacle_msg_count, planning_msg_count):
                    localization_msgs.append(msg_new)
                    localization_msg_count += 1
                else:
                    localization_msgs[-1] = msg_new
            else:
                if localization_msg_count == min(light_msg_count, obstacle_msg_count, planning_msg_count):
                    localization_msgs.append(msg_new)
                    localization_msg_count += 1
                else:
                    localization_msgs[-1] = msg_new

    if test_system != 'obstacle':
        msg_count = min(localization_msg_count, light_msg_count, obstacle_msg_count, planning_msg_count,
                        prediction_msg_count)
    else:
        msg_count = min(localization_msg_count, light_msg_count, obstacle_msg_count, planning_msg_count)
    localization_msgs = localization_msgs[:msg_count]
    light_msgs = light_msgs[:msg_count]
    obstacle_msgs = obstacle_msgs[:msg_count]
    planning_msgs = planning_msgs[:msg_count]
    if test_system != 'obstacle':
        prediction_msgs = prediction_msgs[:msg_count]

    return localization_msgs, light_msgs, obstacle_msgs, prediction_msgs, planning_msgs



def get_record_length(reader_path):
    # print(reader_path)
    freader = record.RecordReader(reader_path)
    time.sleep(1)
    # print('+' * 80)
    # print('+++ Begin to read +++')

    timestamps = []
    for channelname, msg, datatype, timestamp in freader.read_messages():
        timestamp = timestamp / 1000000000.0
        timestamp = "{:.6f}".format(timestamp)
        # print(timestamp, type(timestamp))
        timestamps.append(timestamp)

    # print(timestamps[0], timestamps[1])
    length = get_sequence_length(timestamps[0], timestamps[-1])
    start_time = None
    end_time = None
    if length > 3:
        start_time = timestamps[0]
        for i in range(1, len(timestamps)):
            if get_sequence_length(start_time, timestamps[i]) > 3:
                end_time = timestamps[i]
                break
    # print(length)
    return length, start_time, end_time


def get_sequence_length(timestamp_start, timestamp_end):
    # print(str(timestamp_start)[:19])
    date_start = datetime.fromtimestamp(float(str(timestamp_start)[:17]))
    # print(date_start)
    date_end = datetime.fromtimestamp(float(str(timestamp_end)[:17]))
    # print(date_end)
    length = date_end - date_start
    length = length.total_seconds()
    # print(date_start, date_end, length)
    return length


def get_clip_msgs(clip):
    localization_msgs = []
    light_msgs = []
    obstacle_msgs = []
    prediction_msgs = []
    planning_msgs = []

    for channelname, msg, datatype, timestamp in clip:
        if channelname == '/apollo/localization/pose':
            msg_new = ADCTrajectory()
            msg_new.ParseFromString(msg)
            localization_msgs.append(msg_new)
        elif channelname == '/apollo/perception/obstacles':
            msg_new = PerceptionObstacles()
            msg_new.ParseFromString(msg)
            obstacle_msgs.append(msg_new)
        elif channelname == '/apollo/perception/traffic_light':
            msg_new = TrafficLightDetection()
            msg_new.ParseFromString(msg)
            light_msgs.append(msg_new)
        elif channelname == '/apollo/prediction':
            msg_new = PredictionObstacles()
            msg_new.ParseFromString(msg)
            prediction_msgs.append(msg_new)
        elif channelname == '/apollo/planning':
            msg_new = ADCTrajectory()
            msg_new.ParseFromString(msg)
            planning_msgs.append(msg_new)

    return localization_msgs, light_msgs, obstacle_msgs, prediction_msgs, planning_msgs

def write_record(reader_path, len_split=1):
    freader = record.RecordReader(reader_path)
    time.sleep(1)
    # print('+' * 80)
    # print('+++ Begin to read +++')

    # channels = freader.get_channellist()
    start_time = None
    end_time = None
    # count = 0
    for channelname, msg, datatype, timestamp in freader.read_messages():
        time_date = datetime.fromtimestamp(long(str(timestamp)[:14]))
        if not start_time or time_date == end_time:
            start_time = time_date
            end_time = time_date + timedelta(seconds=len_split)
            print(start_time, end_time)
            fwriter = record.RecordWriter()
            if not fwriter.open(start_time.strftime("%Y-%m-%d-%H-%M-%S") + '.record'):
                print('Failed to open record writer!')
                return

        desc = freader.get_protodesc(channelname)
        fwriter.write_channel(channelname, datatype, desc)
        fwriter.write_message(channelname, msg, timestamp)

        # timestamps.append(timestamp)
        if timestamp >= start_time and timestamp < end_time:
            desc = freader.get_protodesc(channelname)
            fwriter.write_channel(channelname, datatype, desc)
            fwriter.write_message(channelname, msg, timestamp)
        elif timestamp == end_time:
            break

    print("finish split")





def get_record_segment(source_record, save_path, start_time, end_time, length=3,
                       check_channel='localization', additional_length=1):
    start_time -= additional_length
    length += additional_length
    print(start_time, end_time)
    # if start_time == end_time:
    #     return

    fwriter = record.RecordWriter()
    if not fwriter.open(save_path + '.record'):
        print('Failed to open record writer!')
        return


    freader = record.RecordReader(source_record)
    time.sleep(1)

    routing_request_msg = None
    routing_response_msg = None

    tf_static_msg = None
    write_status = 0  # 0: not start; 1: writing;
    channel_list = []

    necessary_msgs = 0
    # get tf_static_msg
    for channelname, msg, datatype, timestamp in freader.read_messages():

        if necessary_msgs < 3:
            if channelname == '/tf_static':
                tf_static_msg = msg
                necessary_msgs += 1
            elif channelname == '/apollo/routing_request':
                routing_request_msg = msg
                necessary_msgs += 1
            elif channelname == '/apollo/routing_response':
                routing_response_msg = msg
                necessary_msgs += 1
        else:
            break

    freader = record.RecordReader(source_record)
    time.sleep(1)
    # print('+' * 80)
    # print('+++ Begin to read +++')
    for channelname, msg, datatype, timestamp in freader.read_messages():
        if datatype == 'apollo.localization.LocalizationEstimate':
            msg_new = LocalizationEstimate()
            msg_new.ParseFromString(msg)

            current_time = msg_new.header.timestamp_sec
            print(current_time)

    freader = record.RecordReader(source_record)
    time.sleep(1)
    count = 0
    for channelname, msg, datatype, timestamp in freader.read_messages():
        # print (count)
        count += 1
        # if count < 100:
        #     count += 1
        #     print(channelname, datatype)
        # else:
        #     break
        if write_status == 1:
            if channelname not in channel_list:
                channel_list.append(channelname)
                desc = freader.get_protodesc(channelname)

                fwriter.write_channel(channelname, datatype, desc)
            fwriter.write_message(channelname, msg, timestamp)
            if datatype == 'apollo.localization.LocalizationEstimate':
                msg_new = LocalizationEstimate()
                msg_new.ParseFromString(msg)
                current_time = msg_new.header.timestamp_sec
                if check_channel == 'localization':
                    if current_time == end_time or get_sequence_length(start_time, current_time) > length:
                        print("end: ", current_time, end_time)
                        # if channelname not in channel_list:
                        #     channel_list.append(channelname)
                        #     desc = freader.get_protodesc(channelname)
                        #     fwriter.write_channel(channelname, datatype, desc)
                        # fwriter.write_message(channelname, msg, timestamp)
                        break

        elif datatype == 'apollo.localization.LocalizationEstimate':
            msg_new = LocalizationEstimate()
            msg_new.ParseFromString(msg)

            current_time = msg_new.header.timestamp_sec

            if current_time >= start_time:
                print("start:", current_time, start_time)
                write_status = 1
                start_timestamp = timestamp
                # write tf_static_msg
                desc = freader.get_protodesc('/tf_static')

                if tf_static_msg:
                    fwriter.write_channel('/tf_static', 'apollo.transform.TransformStampeds', desc)
                    # print(tf_static_msg)
                    fwriter.write_message('/tf_static', tf_static_msg, start_timestamp)

                if routing_request_msg:
                    desc = freader.get_protodesc('/apollo/routing_request')
                    fwriter.write_channel('/apollo/routing_request', 'apollo.routing.RoutingRequest', desc)
                    # print(tf_static_msg)
                    fwriter.write_message('/apollo/routing_request', routing_request_msg, start_timestamp)

                if routing_response_msg:
                    desc = freader.get_protodesc('/apollo/routing_response')
                    fwriter.write_channel('/apollo/routing_response', 'apollo.routing.RoutingResponse', desc)
                    # print(tf_static_msg)
                    fwriter.write_message('/apollo/routing_response', routing_response_msg, start_timestamp)

                if channelname not in channel_list:
                    channel_list.append(channelname)
                    desc = freader.get_protodesc(channelname)
                    fwriter.write_channel(channelname, datatype, desc)
                fwriter.write_message(channelname, msg, timestamp)
