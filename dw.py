#
# Copyright (c) 2019-2020 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

from websocket import create_connection
from enum import Enum
import json
import logging
import argparse
import sys
log = logging.getLogger(__name__)


class CoordType(Enum):
    Unity = 1
    Northing = 2
    Latitude = 3


class Connection:
    def __init__(self, ip='localhost', port="8888"):
        """
        simulator: is an lgsvl.Simulator object
        ego_agent: an lgsvl.EgoVehicle object, this is intended to be used with a vehicle equipped with Apollo 5.0
        ip: address of the machine where the Apollo stack is running
        port: the port number for Dreamview
        """
        self.url = "ws://" + ip + ":" + port + "/websocket"
        self.ws = create_connection(self.url)



    def enable_module(self, module):
        """
        module is the name of the Apollo 5.0 module as seen in the "Module Controller" tab of Dreamview
        """
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "START_MODULE", "value": module})
        )
        return

    def disable_module(self, module):
        """
        module is the name of the Apollo 5.0 module as seen in the "Module Controller" tab of Dreamview
        """
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "STOP_MODULE", "value": module})
        )
        return

    def set_hd_map(self, hd_map):
        """
        Folders in /apollo/modules/map/data/ are the available HD maps
        Map options in Dreamview are the folder names with the following changes:
            - underscores (_) are replaced with spaces
            - the first letter of each word is capitalized

        hd_map parameter is the modified folder name.
        hd_map should match one of the options in the right-most drop down in the top-right corner of Dreamview.
        """

        word_list = []
        for s in hd_map.split('_'):
            word_list.append(s[0].upper() + s[1:])

        mapped_map = ' '.join(word_list)
        print(mapped_map)

        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "CHANGE_MAP", "value": mapped_map})
        )

        # if not self.get_current_map() == mapped_map:
        #     folder_name = hd_map.replace(" ", "_")
        #     error_message = (
        #         "HD Map {0} was not set. Verify the files exist in "
        #         "/apollo/modules/map/data/{1} and restart Dreamview -- Aborting..."
        #     )
        #     log.error(
        #         error_message.format(
        #             mapped_map, folder_name
        #         )
        #     )
        #     sys.exit(1)
        return

    def disable_module(self, module):
        """
        module is the name of the Apollo 5.0 module as seen in the "Module Controller" tab of Dreamview
        """
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "STOP_MODULE", "value": module})
        )
        return


    def set_setup_mode(self, mode):
        """
        mode is the name of the Apollo 5.0 mode as seen in the left-most drop down in the top-right corner of Dreamview
        """
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "CHANGE_MODE", "value": mode})
        )
        return

    def get_module_status(self):
        """
        Returns a dict where the key is the name of the module and value is a bool based on the module's current status
        """
        self.reconnect()
        data = json.loads(
            self.ws.recv()
        )  # This first recv() call returns the SimControlStatus in the form '{"enabled":false,"type":"SimControlStatus"}'
        while data["type"] != "HMIStatus":
            data = json.loads(self.ws.recv())

        # The second recv() call also contains other information:
        #   the current map, vehicle, and mode:
        #       data["data"]["currentMap"], data["data"]["currentVehicle"], data["data"]["currentMode"]
        #
        #   the available maps, vehicles, and modes:
        #       data["data"]["maps"], data["data"]["vehicles"], data["data"]["modes"]
        #
        #   the status of monitors components:
        #       data["data"]["monitoredComponents"]

        return data["data"]["modules"]


    def reconnect(self):
        """
        Closes the websocket connection and re-creates it so that data can be received again
        """
        self.ws.close()
        self.ws = create_connection(self.url)
        return



class WaitApolloError(Exception):
    """
    Raised when Apollo control message is not received in time
    """

    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument("-m", "--module", action="store", type=str, required=True,
                        help='apollo module name')
    parser.add_argument("-p", "--hdmap", action="store", type=str, help='hdmap name')
    parser.add_argument(
        "-d", "--disable", action="store_const", const=True,
        help="Show all lane ids in map")

    args = parser.parse_args()
    # print(args.enable)
    dw = Connection()


    if args.disable:
        if args.module == 'Traffic':
            dw.disable_module("Traffic Light")
        else:
            dw.disable_module(args.module)
    else:
        if args.hdmap:
            if args.hdmap == "san":
                hdmap = "San Francisco"
            else:
                hdmap = args.hdmap
            dw.set_hd_map(hdmap)
        if args.module == 'Traffic':
            dw.enable_module("Traffic Light")
        else:
            dw.enable_module(args.module)