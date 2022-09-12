import math
import os
import time
from datetime import datetime
import pickle
import numpy as np
# from cyber_py import record
from map_parsing import Map, near_crosswalk, near_junction, near_stop_sign, is_on_lane, is_on_crosswalk
from utils import get_clip_msgs, get_alignment_messages, vec_denoise, cal_similarity, check_direction, \
    get_sequence_length, get_majority, vec_denoise_v3, get_record_segment

def create_action_vecs(localization_msgs, prediction_msgs, planning_msgs):
    i = 0
    ego_actions = []
    obstacle_actions = []

    scenario = []
    # stage_type = []
    decision = []
    stop_reason = []
    estop_reason = []

    is_static = []
    priority = []
    obstacle_decisions = []
    obstacle_types = []
    priority_set = []
    obstacle_decisions_set = []
    obstacle_types_set = []

    for localization_msg, prediction_msg, planning_msg in zip(localization_msgs, prediction_msgs, planning_msgs):
        # ego-vehicle
        current_scenario = planning_msg.debug.planning_data.scenario.scenario_type
        current_decision = -1

        if planning_msg.decision.main_decision.stop.ByteSize() > 0:
            current_decision = 0
        elif planning_msg.decision.main_decision.cruise.ByteSize() > 0:
            current_decision = 1
        elif planning_msg.decision.main_decision.estop.ByteSize() > 0:
            current_decision = 2
        elif planning_msg.decision.main_decision.change_lane.ByteSize() > 0:
            current_decision = 3
        elif planning_msg.decision.main_decision.mission_complete.ByteSize() > 0:
            current_decision = 4
        elif planning_msg.decision.main_decision.not_ready.ByteSize() > 0:
            current_decision = 5
        elif planning_msg.decision.main_decision.parking.ByteSize() > 0:
            current_decision = 6

        current_stop_reason = 0
        current_estop_reason = 0
        if current_decision == 0:
            current_stop_reason = planning_msg.decision.main_decision.stop.reason_code
            if current_stop_reason >= 100 and current_stop_reason < 200:
                current_stop_reason -= 94
            elif current_stop_reason >= 200:

                current_stop_reason -= 194

        elif current_decision == 2:
            current_estop_reason = planning_msg.decision.main_decision.estop.reason_code

        scenario.append(current_scenario)
        decision.append(current_decision)
        stop_reason.append(current_stop_reason)
        estop_reason.append(current_estop_reason)
        # ego_actions.append([current_scenario, current_decision, current_estop_reason, current_stop_reason])

        # obstacles
        prediction_obstacles = prediction_msg.prediction_obstacle
        action_matrix = []
        current_static = []
        current_priority = []
        current_obs_decision = []
        current_obs_type = []

        for prediction_obstacle in prediction_obstacles:

            perception_obstacle = prediction_obstacle.perception_obstacle


            #     for perception_obstacle in prediction_obstacle:
            obstacle_id = perception_obstacle.id
            obstacle_type = perception_obstacle.type
            obstacle_priority = prediction_obstacle.priority.priority
            obstacle_status = prediction_obstacle.is_static
            current_static.append(obstacle_status)
            current_priority.append(obstacle_priority)
            current_obs_type.append(obstacle_type)
            #
            action_matrix.append({'id': obstacle_id, 'type': obstacle_type, 'priority': obstacle_priority,
                                  'status': obstacle_status})
            #
            obstacle_decision = planning_msg.decision.object_decision.decision



            for d in obstacle_decision:
                current_obs_decision.append(str(d.object_decision)[1:].split(' ')[0])



        if sum(current_static) > 0:
            is_static.append(1)
        else:
            is_static.append(0)

        priority.append(set(current_priority))
        obstacle_decisions.append(current_obs_decision)
        obstacle_types.append(set(current_obs_type))

        priority_set += current_priority
        obstacle_decisions_set += current_obs_decision
        obstacle_types_set += current_obs_type


    priority_set = list(set(priority_set))
    obstacle_decisions_set = list(set(obstacle_decisions_set))
    obstacle_types_set = list(set(obstacle_types_set))


    if len(priority_set) != 0:
        priority_vec = []
        for p in priority:
            vec = [0 for _ in range(len(priority_set)+1)]
            for v in p:
                vec[priority_set.index(v)] = 1
            priority_vec.append(vec)
        priority_vec = np.vstack(priority_vec)

        obs_decisions_vec = []
        for d in obstacle_decisions:
            vec = [0 for _ in range(len(obstacle_decisions_set)+1)]
            for v in d:
                vec[obstacle_decisions_set.index(v)] = 1
            obs_decisions_vec.append(vec)
        obs_decisions_vec = np.vstack(obs_decisions_vec)

        obs_types_vec = []
        for t in obstacle_types:
            vec = [0 for _ in range(len(obstacle_types_set)+1)]
            for v in t:
                vec[obstacle_types_set.index(v)] = 1
            obs_types_vec.append(vec)
        obs_types_vec = np.vstack(obs_types_vec)

        obstacle_actions = np.concatenate([priority_vec, obs_decisions_vec, obs_types_vec], axis=1)
    else:
        obstacle_actions = np.array([])

    scenario_vec = []
    if len(scenario) != 0:
        for s in scenario:
            vec = [0 for _ in range(13)]
            vec[s] = 1
            scenario_vec.append(vec)
        scenario_vec = np.vstack(scenario_vec)

        decision_vec = []
        for d in decision:
            vec = [0 for _ in range(7)]
            vec[d] = 1
            decision_vec.append(vec)
        decision_vec = np.vstack(decision_vec)

        stop_reason_vec = []
        for s in stop_reason:

            vec = [0 for _ in range(18)]
            vec[s] = 1
            stop_reason_vec.append(vec)
        stop_reason_vec = np.vstack(stop_reason_vec)

        estop_reason_vec = []
        for e in estop_reason:
            vec = [0 for _ in range(6)]
            vec[e] = 1
            estop_reason_vec.append(vec)
        estop_reason_vec = np.vstack(estop_reason_vec)

        ego_actions = np.concatenate([scenario_vec, decision_vec, stop_reason_vec, estop_reason_vec], axis=1)
    else:
        ego_actions = np.array([])


    return ego_actions.tolist(), obstacle_actions.tolist()


def create_actor_vecs(localization_msgs, obstacle_msgs, map):
    has_vehicle = []
    has_bicycle = []
    has_pedestrian = []
    has_other = []

    # position
    has_back_vehicle = []
    has_back_person = []
    has_side_vehicle = []
    has_side_person = []
    has_on_road_person = []
    has_on_crosswalk_person = []
    has_same_direction_vehicle = []
    has_opposing_direction_vehicle = []
    has_crossing_direction_vehicle = []
    # count >5?
    # vehicle_num = []
    # pedestrian_num = []

    for localization_msg, obstacle_msg in zip(localization_msgs, obstacle_msgs):
        self_pos = (localization_msg.pose.position.x,
                    localization_msg.pose.position.y, localization_msg.pose.position.z)
        self_heading = math.degrees(localization_msg.pose.heading)
        obs_num = len(obstacle_msg.perception_obstacle)
        current_vehicle = 0
        current_bicycle = 0
        current_pedestrian = 0
        # current_trafficcone = 0
        current_other = 0

        current_obs_dis = []
        current_vehicle_pos = []
        current_person_pos = []
        current_vehicle_loc = []
        current_person_loc = []
        current_vehicle_direction = []

        current_vehicle_headings = []
        current_vehicle_positions = []

        for i in range(obs_num):
            if obstacle_msg.perception_obstacle[i].type == 3:
                current_pedestrian += 1
            elif obstacle_msg.perception_obstacle[i].type == 4:
                current_bicycle += 1
            elif obstacle_msg.perception_obstacle[i].type == 5:
                current_vehicle += 1
            else:
                current_other += 1

            obs_pos = (obstacle_msg.perception_obstacle[i].position.x,
                       obstacle_msg.perception_obstacle[i].position.y,
                       obstacle_msg.perception_obstacle[i].position.z)
            obs_heading = math.degrees(obstacle_msg.perception_obstacle[i].theta)
            current_vehicle_positions.append(obs_pos)
            current_vehicle_headings.append(obs_heading)

            current_obs_dis.append(
                (math.sqrt((obs_pos[0] - self_pos[0]) ** 2 + (obs_pos[1] - self_pos[1]) ** 2)))

            if obstacle_msg.perception_obstacle[i].type == 3:
                #     current_person_loc.append(is_in_front(self_pos, obs_pos, self_heading))

                if not is_on_lane(obs_pos, map):
                    current_person_pos.append("side")
                elif is_on_crosswalk(obs_pos, map):
                    current_person_pos.append("crosswalk")
                else:
                    current_person_pos.append("road")
            else:
                # current_vehicle_loc.append(is_in_front(self_pos, obs_pos, self_heading))
                # current_vehicle_direction.append(check_direction(self_heading, obs_heading))
                if not is_on_lane(obs_pos, map):
                    current_vehicle_pos.append("side")
                else:
                    current_vehicle_pos.append("road")

        if current_vehicle > 0:
            has_vehicle.append(1)
        else:
            has_vehicle.append(0)

        if current_pedestrian > 0:
            has_pedestrian.append(1)
        else:
            has_pedestrian.append(0)

        if current_bicycle > 0:
            has_bicycle.append(1)
        else:
            has_bicycle.append(0)

        if current_other > 0:
            has_other.append(1)
        else:
            has_other.append(0)

        if False in current_vehicle_loc:
            has_back_vehicle.append(1)
        else:
            has_back_vehicle.append(0)

        if False in current_person_loc:
            has_back_person.append(1)
        else:
            has_back_person.append(0)


        if "side" in current_vehicle_pos:
            has_side_vehicle.append(1)
        else:
            has_side_vehicle.append(0)

        if "side" in current_person_pos:
            has_side_person.append(1)
        else:
            has_side_person.append(0)

        if "crosswalk" in current_person_pos:
            has_on_crosswalk_person.append(1)
        else:
            has_on_crosswalk_person.append(0)

        if "road" in current_person_pos:
            has_on_road_person.append(1)
        else:
            has_on_road_person.append(0)

        for i in range(len(current_vehicle_headings)):
            direction = check_direction(self_heading, current_vehicle_headings[i])
            current_vehicle_direction.append(direction)

        if 2 in current_vehicle_direction:
            has_opposing_direction_vehicle.append(1)
        else:
            has_opposing_direction_vehicle.append(0)

        if 3 in current_vehicle_direction:
            has_crossing_direction_vehicle.append(1)
        else:
            has_crossing_direction_vehicle.append(0)

    if len(has_vehicle) != 0:
        has_vehicle = np.vstack(has_vehicle)
        has_bicycle = np.vstack(has_bicycle)
        has_pedestrian = np.vstack(has_pedestrian)
        has_other = np.vstack(has_other)
        # vehicle_num = np.vstack(vehicle_num)
        # pedestrian_num = np.vstack(pedestrian_num)
        # max_distance = np.vstack(max_distance)
        # min_distance = np.vstack(min_distance)
        # has_back_vehicle = np.vstack(has_back_vehicle)
        # has_back_person = np.vstack(has_back_person)
        # has_side_vehicle = np.vstack(has_side_vehicle)
        # has_side_person = np.vstack(has_side_person)
        has_on_road_person = np.vstack(has_on_road_person)
        has_on_crosswalk_person = np.vstack(has_on_crosswalk_person)
        # has_same_direction_vehicle = np.vstack(has_same_direction_vehicle)
        has_opposing_direction_vehicle = np.vstack(has_opposing_direction_vehicle)
        has_crossing_direction_vehicle = np.vstack(has_crossing_direction_vehicle)

        actor_vec = np.hstack([has_vehicle, has_bicycle, has_pedestrian, has_other,
                               has_on_road_person, has_on_crosswalk_person,
                               has_opposing_direction_vehicle, has_crossing_direction_vehicle])
    else:
        actor_vec = np.zeros((len(localization_msgs), 8))

    return actor_vec.tolist()


def create_scene_vecs(localization_msgs, light_msgs, map):
    signal_color = []
    signal_type = []
    subsignal_type = []
    signal_distance = []
    signal_position = []
    has_crosswalk = []
    has_intersection = []
    has_stop_sign = []

    # create scene vector [traffic light colors, has_intersection, has_crosswalk, has_stop_sign]
    for localization_msg, light_msg in zip(localization_msgs, light_msgs):
        self_pos = (localization_msg.pose.position.x,
                    localization_msg.pose.position.y, localization_msg.pose.position.z)

        has_crosswalk.append(near_crosswalk(self_pos, map))
        has_intersection.append(near_junction(self_pos, map))
        has_stop_sign.append(near_stop_sign(self_pos, map))
        if not light_msg.contain_lights:
            signal_color.append(-1)
            signal_type.append(-1)
            signal_distance.append(-1)
            signal_position.append(-1)
            subsignal_type.append(-1)
        else:
            status = light_msg.traffic_light[0].color
            signal_color.append(status)

            detected_signal = map.signals[light_msg.traffic_light[0].id]
            detected_signal_type = detected_signal["type"]
            signal_type.append(detected_signal_type)

            detected_subsignal_type = detected_signal["subsignal_type"]
            subsignal_type.append(detected_subsignal_type)

    signal_color_vec = []
    for status in signal_color:
        if status == -1:
            signal_color_vec.append([1, 0, 0, 0, 0, 0])
        elif status == 0:
            signal_color_vec.append([0, 1, 0, 0, 0, 0])
        elif status == 1:
            signal_color_vec.append([0, 0, 1, 0, 0, 0])
        elif status == 2:
            signal_color_vec.append([0, 0, 0, 1, 0, 0])
        elif status == 3:
            signal_color_vec.append([0, 0, 0, 0, 1, 0])
        elif status == 4:
            signal_color_vec.append([0, 0, 0, 0, 0, 1])

    signal_type_vec = []
    for detected_signal_type in signal_type:
        if detected_signal_type == -1:
            signal_type_vec.append([1, 0, 0, 0, 0, 0, 0])
        elif detected_signal_type == 1:
            signal_type_vec.append([0, 1, 0, 0, 0, 0, 0])
        elif detected_signal_type == 2:
            signal_type_vec.append([0, 0, 1, 0, 0, 0, 0])
        elif detected_signal_type == 3:
            signal_type_vec.append([0, 0, 0, 1, 0, 0, 0])
        elif detected_signal_type == 4:
            signal_type_vec.append([0, 0, 0, 0, 1, 0, 0])
        elif detected_signal_type == 5:
            signal_type_vec.append([0, 0, 0, 0, 0, 1, 0])
        elif detected_signal_type == 6:
            signal_type_vec.append([0, 0, 0, 0, 0, 0, 1])

    subsignal_type_vec = []
    for detected_subsignal_type in subsignal_type:
        if detected_subsignal_type == -1:
            subsignal_type_vec.append([1, 0, 0, 0])
        elif detected_subsignal_type == 1:
            subsignal_type_vec.append([0, 1, 0, 0])
        elif detected_subsignal_type == 2:
            subsignal_type_vec.append([0, 0, 1, 0])
        elif detected_subsignal_type == 3:
            subsignal_type_vec.append([0, 0, 0, 1])
        # break

    if signal_color:
        signal_color_vec = np.vstack(signal_color_vec)
        signal_type_vec = np.vstack(signal_type_vec)
        subsignal_type_vec = np.vstack(subsignal_type_vec)
        has_crosswalk_vec = np.array(has_crosswalk).reshape(-1, 1)
        has_stop_sign_vec = np.array(has_stop_sign).reshape(-1, 1)
        has_intersection_vec = np.array(has_intersection).reshape(-1, 1)
        scene_vec = np.hstack([signal_color_vec, signal_type_vec, subsignal_type_vec, has_crosswalk_vec,
                               has_stop_sign_vec, has_intersection_vec])


    return scene_vec.tolist()


def record2vec(source_record, map_name, regression=0, save_path=None, test_system=None, save_file=True):
    localization_msgs, light_msgs, obstacle_msgs, prediction_msgs, planning_msgs = get_alignment_messages(source_record,
                                                                                                          test_system)

    map = Map('maps/{}.bin'.format(map_name))
    map.parse()

    if len(localization_msgs) == 0:
        return

    if not test_system:
        scene_vec = create_scene_vecs(localization_msgs, light_msgs, map)

        actor_vec = create_actor_vecs(localization_msgs, obstacle_msgs, map)

        ego_action_vec, obs_action_vec = create_action_vecs(localization_msgs, prediction_msgs, planning_msgs)

        scene_vec = vec_denoise_v3(scene_vec, 3)
        actor_vec = vec_denoise_v3(actor_vec, 3)
        ego_action_vec = vec_denoise_v3(ego_action_vec, 3)
        obs_action_vec = vec_denoise_v3(obs_action_vec, 3)

        record_data = {
            'scene_vec': scene_vec,
            'actor_vec': actor_vec,
            'ego_action_vec': ego_action_vec,
            'obs_action_vec': obs_action_vec,
            'localization_msgs': localization_msgs,
            'light_msgs': light_msgs,
            'prediction_msgs': prediction_msgs,
            'planning_msgs': planning_msgs,
            'obstacle_msgs': obstacle_msgs
        }
    elif test_system == 'signal':
        scene_vec = create_scene_vecs(localization_msgs, light_msgs, map)
        scene_vec = vec_denoise_v3(scene_vec, 3)
        record_data = {
            'scene_vec': scene_vec,
            'localization_msgs': localization_msgs,
            'light_msgs': light_msgs,
            'prediction_msgs': prediction_msgs,
            'planning_msgs': planning_msgs,
            'obstacle_msgs': obstacle_msgs
        }

    record_time = [0]
    segment_start_time = record_data['localization_msgs'][0].header.timestamp_sec
    record_timestamp = [segment_start_time]
    for i in range(1, len(record_data['localization_msgs'])):
        record_time.append(get_sequence_length(segment_start_time,
                                               record_data['localization_msgs'][i].header.timestamp_sec))
        record_timestamp.append(record_data['localization_msgs'][i].header.timestamp_sec)

    record_data['time'] = record_time
    record_data['timestamp'] = record_timestamp

    if save_file:
        if not save_path:
            save_name = source_record.split('/')[-1]
            if regression == 0:
                save_path = 'data/record_data/original/{}.pickle'.format(save_name)
            else:
                save_path = 'data/record_data/regression/{}.pickle'.format(save_name)

        with open(save_path, 'wb') as handle:
            pickle.dump(record_data, handle, protocol=pickle.HIGHEST_PROTOCOL)

    return record_data


def clip_segment(segment, recording_time, kept_length=3):
    clip_start = segment[0]
    for i in range(segment[0] + 1, segment[1]):
        if recording_time[i] - recording_time[clip_start] >= kept_length:
            clip_end = i
            break

    return (clip_start, clip_end)


def record2segments(record_data, test_system='obstacle', kept_length=3):
    segment_data = {}

    if test_system == 'obstacle':
        segments_ind, vecs = split_record(record_data['actor_vec'])
    elif test_system == 'signal':
        segments_ind, vecs = split_record(record_data['scene_vec'])
    elif test_system == 'prediction':
        segments_ind, vecs = split_record(record_data['obs_action_vec'])
    elif test_system == 'planning':
        segments_ind, vecs = split_record(record_data['ego_action_vec'])

    segments_new = []
    vecs_new = []

    count = 0
    for segment, vec in zip(segments_ind, vecs):
        if record_data['time'][segment[1]] - record_data['time'][segment[0]] >= 1:
            segments_new.append((count, segment))
            vecs_new.append(vec)
        count += 1

    segments = segments_new
    vecs = vecs_new

    segment_data['segments'] = segments
    segment_data['vecs'] = vecs
    segment_data['segments_kept'] = []

    segments_length = []
    segments_reduced = []
    segments_length_reduced = []
    segments_vecs_reduced = []


    for segment_info, vec in zip(segments, vecs):
        idx = segment_info[0]
        segment = segment_info[1]
        length = get_sequence_length(record_data['localization_msgs'][segment[0]].header.timestamp_sec,
                                     record_data['localization_msgs'][segment[1] - 1].header.timestamp_sec)

        if idx == 0:
            segments_length.append(length)
        else:
            segments_length.append(length+1)

        if kept_length != -1 and length >= kept_length:
            segment_kept = clip_segment(segment, record_data['time'], kept_length)
        else:
            segment_kept = segment
        segment_data['segments_kept'].append((idx, segment_kept))

        if length >= 1 and vec not in segments_vecs_reduced:
            if length <= kept_length:
                segments_length_reduced.append(length+1)
                segments_reduced.append((idx, segment_kept))
            else:
                segments_length_reduced.append(kept_length+1)
                segments_reduced.append((idx, segment_kept))

            segments_vecs_reduced.append(vec)

    segment_data['segments_length'] = segments_length
    segment_data['segments_length_reduced'] = segments_length_reduced
    segment_data['segments_reduced'] = segments_reduced
    segment_data['segments_vecs_reduced'] = segments_vecs_reduced



    return segment_data


def split_record(record_vec, sim_threshold=1):
    clips = []
    vecs = []
    current_scenario_start = 0
    current_scenario_end = 0


    for i in range(1, len(record_vec)):
        if record_vec[i] == record_vec[i-1]:
            current_scenario_end = i
        else:
            clips.append((current_scenario_start, current_scenario_end + 1))
            vecs.append(record_vec[current_scenario_end])
            current_scenario_start = i
            current_scenario_end = i

    vecs.append(record_vec[current_scenario_end])
    clips.append((current_scenario_start, current_scenario_end + 1))

    return clips, vecs


def extract_record_data():
    maps = ['gomentum', 'cubetown', 'borregas_ave']


    for map in maps:
        print(map)
        if not os.path.exists('data/record_segments/{}'.format(map)):
            os.mkdir('data/record_segments/{}'.format(map))

        record2vec('data/records/{}.record'.format(map), map, 0)

        file = open('data/record_data/original/{}.record.pickle'.format(map), 'rb')
        record_data = pickle.load(file)
        segments = {}
        for module in ['signal']:
            # for module in ['signal', 'obstacle', 'planning', 'prediction']:
            if not os.path.exists('data/record_segments/{}/{}'.format(map, module)):
                os.mkdir('data/record_segments/{}/{}'.format(map, module))
            if not os.path.exists('data/record_segments/{}/{}/all'.format(map, module)):
                os.mkdir('data/record_segments/{}/{}/all'.format(map, module))
            if not os.path.exists('data/record_segments/{}/{}/reduced'.format(map, module)):
                os.mkdir('data/record_segments/{}/{}/reduced'.format(map, module))
            segments_module = record2segments(record_data, module, kept_length=3)

            segments[module] = segments_module
            for i, segment_ind in enumerate(segments_module['segments']):
                get_record_segment('data/records/{}.record'.format(map),
                                   'data/record_segments/{}/{}/all/segment_{}.record'.format(map, module, i),
                                   record_data['timestamp'][segment_ind[1][0]], record_data['timestamp'][segment_ind[1][1]],
                                   length=999)

            for i, segment_info in enumerate(segments_module['segments_reduced']):
                idx = segment_info[0]
                segment_ind = segment_info[1]
                if idx != 0:
                    get_record_segment('data/records/{}.record'.format(map),
                                       'data/record_segments/{}/{}/reduced/segment_{}.record'.format(map, module, idx),
                                       record_data['timestamp'][segment_ind[1][0]], record_data['timestamp'][segment_ind[1][1]])
                else:
                    get_record_segment('data/records/{}.record'.format(map),
                                       'data/record_segments/{}/{}/reduced/segment_{}.record'.format(map, module, idx),
                                       record_data['timestamp'][segment_ind[1][0]], record_data['timestamp'][segment_ind[1][1]],
                                       additional_length=0)

        with open('data/record_data/original/{}_segments.pickle'.format(map), 'wb') as handle:
            pickle.dump(segments, handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    extract_record_data()