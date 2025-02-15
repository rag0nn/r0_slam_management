import yaml
from sensor_msgs.msg import CameraInfo


CONFIG_PATH = "/home/rag0n/Desktop/r0_workspace/my_configs/const_params.yaml"

def get_params():
    with open(CONFIG_PATH, "r") as file:
        params = yaml.safe_load(file)
    return params

def load_rgb_params():
    """
    return:
     :topic:
     :pub_periot:
     :pub_queue_size:
     :listen_periot:
     :frame_id:
     :data_path:
    """
    data = get_params()
    return [
        data["rgb"]["topic"],
        data["rgb"]["publish_periot"],
        data["rgb"]["publish_queue_size"],
        data["rgb"]["listen_periot"],
        data["rgb"]["frame_id"],
        data["rgb"]["data_path"],
        data["rgb"]["typeruntime"],
        data["rgb"]["typeshowtime"],
    ]

def load_depth_params():
    """
    return:
     :topic:
     :pub_periot:
     :pub_queue_size:
     :listen_periot:
     :frame_id:
     :data_path:
    """
    data = get_params()
    return [
        data["depth"]["topic"],
        data["depth"]["publish_periot"],
        data["depth"]["publish_queue_size"],
        data["depth"]["listen_periot"],
        data["depth"]["frame_id"],
        data["depth"]["data_path"],
        data["depth"]["typeruntime"],
        data["depth"]["typeshowtime"],
    ]

def load_caminfo_params():
    """
    return:
        :topic:
        :pub_periot:
        :pub_queue_size:
        :listen_periot:
        :frame_id:
        :CameraInfo:
    """
    data = get_params()
    caminfo = CameraInfo()
    caminfo.distortion_model = data["caminfo"]["distortion_model"]
    caminfo.d = data["caminfo"]["distortion_coefficients"]
    caminfo.k = data["caminfo"]["intrinsic_matrix"]
    caminfo.width = data["caminfo"]["resolution"]["width"]
    caminfo.height = data["caminfo"]["resolution"]["height"]
    caminfo.r = data["caminfo"]["r"]
    caminfo.p = data["caminfo"]["p"]
    caminfo.binning_x = data["caminfo"]["binning_x"]
    caminfo.binning_y = data["caminfo"]["binning_y"]
    caminfo.roi.x_offset = data["caminfo"]["roi"]["x_offset"]
    caminfo.roi.y_offset = data["caminfo"]["roi"]["y_offset"]
    caminfo.roi.height = data["caminfo"]["roi"]["height"]
    caminfo.roi.width = data["caminfo"]["roi"]["width"]
    caminfo.roi.do_rectify = data["caminfo"]["roi"]["do_rectify"]
    
    return [
        data["caminfo"]["topic"],
        data["caminfo"]["publish_periot"],
        data["caminfo"]["publish_queue_size"],
        data["caminfo"]["listen_periot"],
        data["caminfo"]["frame_id"],
        caminfo
    ]

def load_rgbd_linked_params():
    return [
        load_rgb_params(),
        load_depth_params(),
        load_caminfo_params(),
    ]

def load_linked_listen_params():
    """
    return:
        :topic_rgb:
        :topic_epth:
        :topic_caminfo:
        :bag_save_path:
    """
    data = get_params()
    return [
        data["rgb"]["topic"],
        data["depth"]["topic"],
        data["caminfo"]["topic"],
        data["bag_save_path"],
    ]

def load_types():
    """
    return:
        :rgb_runtimetype:
        :rgb_showtime_type:
        :depth_runtimetype:
        :depth_showtime_type:
    """
    data = get_params()
    return [
        data["rgb"]["typeruntime"],
        data["rgb"]["typeshowtime"],
        data["depth"]["typeruntime"],
        data["depth"]["typeshowtime"]
    ]





