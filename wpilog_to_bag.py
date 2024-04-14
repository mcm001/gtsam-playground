import rosbag
from wpiutil.log import DataLogReader

wpilog = DataLogReader("/mnt/c/Users/matth/Downloads/Log_24-04-06_09-16-11_q5.wpilog")
bag = rosbag.Bag("out.bag")
