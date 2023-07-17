#!/usr/bin/python
import sys
import rosbag
import rospy
import argparse
import json
import codecs
from PyQt5 import QtCore, QtGui, QtWidgets
from datetime import datetime


def _init_ros_node():
    rospy.init_node("rosbag_to_json_converter", anonymous=True)


def _define_command_options():
    parser = argparse.ArgumentParser(usage="%(prog)s [options]")
    
    parser.add_argument("-a", "--all", dest="all_topics", action="store_true",
                        help="convert all topics", default=False)
    parser.add_argument("-t", "--topic", dest="topic_names", action="append",
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
    
    return files_list


def _get_topics_list(files_list):
    bag = rosbag.Bag(files_list[0])
    topics_list = list(bag.get_type_and_topic_info()[1].keys())

    if not topics_list:
        print("Error: Any topics not found")
        sys.exit()
    
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

    selected_or_not = {}
    for (check_box, topic) in zip(check_boxes, topics_list):
        selected_or_not[topic] = check_box.isChecked()
    
    return selected_or_not


def _get_selected_topics(selected_or_not, args):
    args.topic_names = []

    for topic_name, is_checked in selected_or_not.items():
        if is_checked: args.topic_names.append(topic_name)
    
    if not args.topic_names:
        print("Error: Please select topics")
        sys.exit()


def _interpret_msg(topic, msg, time, data_dict, parent_data_name=""):
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            if not parent_data_name: data_name = s
            else: data_name = ".".join([parent_data_name, s])
            _interpret_msg(topic, val, time, data_dict, data_name)
    except BaseException:
        if not parent_data_name in data_dict: data_dict.setdefault(parent_data_name, {})
        data_dict[parent_data_name][time] = str(msg)


def _default(o):
    print(type(o))
    raise TypeError(repr(o) + "is not JSON serializable")


def _convert_bag_to_json(file, args):
    # load bag file
    try:
        bag = rosbag.Bag(file)
    except Exception as e:
        rospy.logfatal("Failed to load bag file: %s", e)
        exit(1)
    finally:
        rospy.loginfo("Loaded bag file: %s", file)
    
    # read messages
    data_dict = {}
    try:
        for topic, msg, ros_time in bag.read_messages(topics=args.topic_names):
            if not topic in data_dict: data_dict.setdefault(topic, {})

            unix_time = ros_time.to_time()

            _interpret_msg(topic, msg, unix_time, data_dict[topic])
    except Exception as e:
        rospy.logwarn("Failed to read messages: %s", e)
    finally:
        with open("data.json", 'w') as fw:
            json.dump(data_dict, fw, ensure_ascii=False, indent=4, sort_keys=True)

        bag.close()


def _output_json_files(files_list, args):
    args.output_file_format = "%t.json"

    print("Converting...")

    for file in files_list:
        _convert_bag_to_json(file, args)
    
    QtWidgets.QMessageBox.information(
        QtWidgets.QWidget(), 
        "Message",
        "Finished conversion!!"
        )


def main():
    _init_ros_node()

    args = _define_command_options()

    app = _construct_qapplication()

    files_list = _select_bag_files()

    topics_list = _get_topics_list(files_list)

    selected_or_not = _select_topics(app, topics_list)

    _get_selected_topics(selected_or_not, args)

    _output_json_files(files_list, args)


if __name__ == "__main__":
    main()
