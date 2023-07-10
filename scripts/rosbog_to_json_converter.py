#!/usr/bin/python
import sys
import rospy
import argparse
from PyQt5 import QtGui, QtWidgets


def _init_ros_node():
    rospy.init_node("rosbag_to_json_converter", anonymous=True)


def _define_command_options():
    parser = argparse.ArgumentParser(usage="%(prog)s [options]")
    
    parser.add_argument("-a", "--all", dest="all_topics", action="store_true",
                        help="convert all topics", default=False)
    parser.add_argument("-t", "--topic", dest="topic_name", action="append",
                        help="list of topic names", metavar="TOPIC_NAME")
    parser.add_argument("-s", "--start-time", dest="start_time", type=float,
                        help="start time you want to convert")
    parser.add_argument("-e", "--end-time", dest="end_time", type=float,
                        help="end time you want to convert")
    
    return parser.parse_args()


def _construct_qapplication():
    return QtWidgets.QApplication(sys.argv)


def _select_bag_files():
    files = QtWidgets.QFileDialog.getOpenFileNames(caption="Select bag files",
                                                        filter="*.bag")

    files_list = []
    for file in files:
        if type(file) is list:
            for f in file: files_list.append(str(f))
    
    if not files_list:
        print("Error: Please select a bag file")
        sys.exit()
    
    print(files_list)
    return files_list


def main():
    print("rosbag_to_json_converter start")

    _init_ros_node()

    args = _define_command_options()

    app = _construct_qapplication()

    files_list = _select_bag_files()


if __name__ == "__main__":
    main()
