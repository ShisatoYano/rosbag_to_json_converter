#!/usr/bin/python
import sys
import rosbag
import rospy
import argparse
from PyQt5 import QtCore, QtGui, QtWidgets


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


def _get_topics_list(files_list):
    bag = rosbag.Bag(files_list[0])
    topics_list = list(bag.get_type_and_topic_info()[1].keys())

    if not topics_list:
        print("Error: Any topics not found")
        sys.exit()
    
    print(topics_list)
    return topics_list


def _select_topics(app, topics_list):
    window = QtWidgets.QWidget()

    scroll_area = QtWidgets.QScrollArea()
    scroll_area.setWidgetResizable(True)
    scroll_area_widget_contents = QtWidgets.QWidget(scroll_area)
    scroll_area_widget_contents.setGeometry(QtCore.QRect(0, 0, 380, 247))
    scroll_area.setWidget(scroll_area_widget_contents)
    
    layout = QtWidgets.QGridLayout()
    vertical_layout_scroll = QtWidgets.QVBoxLayout(scroll_area_widget_contents)
    layout_index = 0

    label = QtWidgets.QLabel("Select topics to convert json files")
    layout.addWidget(label, layout_index, 0)
    layout_index = layout_index + 1

    check_boxes = []
    for topic in topics_list:
        check_box = QtWidgets.QCheckBox(topic)
        vertical_layout_scroll.addWidget(check_box)
        layout_index = layout_index + 1
        check_boxes.append(check_box)
    
    layout.addWidget(scroll_area)
    ok_button = QtWidgets.QPushButton("OK")
    ok_button.clicked.connect(app.quit)
    layout.addWidget(ok_button, layout_index, 0)
    layout_index = layout_index + 1

    window.setLayout(layout)
    window.setWindowTitle("Select")
    window.show()
    app.exec_()


def main():
    print("rosbag_to_json_converter start")

    _init_ros_node()

    args = _define_command_options()

    app = _construct_qapplication()

    files_list = _select_bag_files()

    topics_list = _get_topics_list(files_list)

    _select_topics(app, topics_list)


if __name__ == "__main__":
    main()
