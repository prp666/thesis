import sys
if sys.prefix == '/home/prp/detection_yolo':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/prp/thesis/thesis_ws/install/sound_detection'
