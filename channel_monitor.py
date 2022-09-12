import sys

sys.path.append("../")
from cyber_py import cyber, record
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles, DESCRIPTOR as obs_desc
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection, DESCRIPTOR as sig_desc
from modules.localization.proto.localization_pb2 import LocalizationEstimate, DESCRIPTOR as loc_desc
from modules.planning.proto.planning_pb2 import ADCTrajectory, DESCRIPTOR as adc_desc
from google.protobuf.json_format import MessageToJson, Parse
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles, DESCRIPTOR as prediction_desc

import pickle
import argparse
from datetime import datetime
import time


def to_seconds(date):
    return time.mktime(date.timetuple())


def obstacle_callback(data):
    """
    Reader message callback.
    """
    now = time.time()

    fwriter.write_message("/apollo/perception/obstacles", data.SerializeToString(),
                          long(now * 1000000000))

    # print("obstacles")
    # msgs.append(MessageToJson(data))


def traffic_light_callback(data):
    now = time.time()
    fwriter.write_message("/apollo/perception/traffic_light", data.SerializeToString(),
                          long(now * 1000000000))
    # print("traffic_light")

def localization_callback(data):
    now = time.time()
    fwriter.write_message("/apollo/localization/pose", data.SerializeToString(),
                          long(now * 1000000000))
    # print("%.2f" % data.header.timestamp_sec)
    # print("localization")

def trajectory_callback(data):
    now = time.time()
    fwriter.write_message("/apollo/planning", data.SerializeToString(),
                          long(now * 1000000000))

def prediction_callback(data):
    now = time.time()
    fwriter.write_message("/apollo/prediction", data.SerializeToString(),
                          long(now * 1000000000))

def obstacle_listener():
    """
    Reader message.
    """
    print ("=" * 120)
    test_node = cyber.Node("listener_1")
    test_node.create_reader("/apollo/perception/obstacles",
                            PerceptionObstacles, obstacle_callback)
    test_node.create_reader("/apollo/perception/traffic_light",
                            TrafficLightDetection, traffic_light_callback)
    test_node.create_reader("/apollo/localization/pose", LocalizationEstimate, localization_callback)
    test_node.create_reader("/apollo/planning", ADCTrajectory, trajectory_callback)
    test_node.create_reader("/apollo/prediction", PredictionObstacles, prediction_callback)

    test_node.spin()
    # test_node.__del__()


def read_message_pickle(filename, proto_class):
    with open(filename, "rb") as f:
        msgs = pickle.load(f)

    print(Parse(msgs[1], proto_class))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument("-n", "--name", action="store", type=str, required=True,
                        help='an integer for the accumulator')
    args = parser.parse_args()
    fwriter = record.RecordWriter()
    localization_data = []
    if not fwriter.open(args.name):
        print('Failed to open record writer!')
    fwriter.write_channel("/apollo/perception/obstacles", "apollo.perception.PerceptionObstacles",
                          obs_desc.serialized_pb)
    fwriter.write_channel("/apollo/perception/traffic_light", "apollo.perception.TrafficLightDetection",
                          sig_desc.serialized_pb)
    fwriter.write_channel("/apollo/localization/pose", "apollo.localization.LocalizationEstimate",
                          loc_desc.serialized_pb)
    fwriter.write_channel("/apollo/planning", "apollo.planning.ADCTrajectory",
                          adc_desc.serialized_pb)

    fwriter.write_channel("/apollo/prediction", "apollo.prediction.PredictionObstacles",
                          prediction_desc.serialized_pb)

    cyber.init()
    obstacle_listener()
    cyber.shutdown()
