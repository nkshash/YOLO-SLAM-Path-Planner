import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import cv2
import torch
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, set_logging
from utils.plots import plot_one_box
from utils.torch_utils import select_device, time_synchronized

class ObjectDetection(Node):
    def __init__(self):
        super().__init__("ObjectDetection")

        self.declare_parameter("weights", "yolov7-tiny.pt", ParameterDescriptor(description="Weights file"))
        self.declare_parameter("conf_thres", 0.25, ParameterDescriptor(description="Confidence threshold"))
        self.declare_parameter("iou_thres", 0.45, ParameterDescriptor(description="IOU threshold"))
        self.declare_parameter("device", "cpu", ParameterDescriptor(description="Name of the device"))
        self.declare_parameter("img_size", 640, ParameterDescriptor(description="Image size"))

        self.weights = self.get_parameter("weights").get_parameter_value().string_value
        self.conf_thres = self.get_parameter("conf_thres").get_parameter_value().double_value
        self.iou_thres = self.get_parameter("iou_thres").get_parameter_value().double_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.img_size = self.get_parameter("img_size").get_parameter_value().integer_value

        self.depth = None
        self.depth_color_map = None
        self.rgb_image = None
        self.intr = None

        self.camera_RGB = False
        self.camera_depth = False

        self.frequency = 20  # Hz
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        self.bridge = CvBridge()
        
        self.rs_sub = self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.rs_callback, 10)
        self.align_depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.align_depth_callback, 10)
        self.intr_sub = self.create_subscription(CameraInfo, 'camera/aligned_depth_to_color/camera_info', self.intr_callback, 10)

        set_logging()
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'
        self.model = attempt_load(self.weights, map_location=self.device)
        stride = int(self.model.stride.max())
        imgsz = check_img_size(self.img_size, s=stride)
        if self.half:
            self.model.half()
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]

        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))
        self.old_img_w = self.old_img_h = imgsz
        self.old_img_b = 1

    def intr_callback(self, cameraInfo):
        if self.intr:
            return
        self.intr = rs.intrinsics()
        self.intr.width = cameraInfo.width
        self.intr.height = cameraInfo.height
        self.intr.ppx = cameraInfo.k[2]
        self.intr.ppy = cameraInfo.k[5]
        self.intr.fx = cameraInfo.k[0]
        self.intr.fy = cameraInfo.k[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            self.intr.model = rs.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            self.intr.model = rs.distortion.kannala_brandt4
        self.intr.coeffs = [i for i in cameraInfo.d]

    def align_depth_callback(self, data):
        self.depth = self.bridge.imgmsg_to_cv2(data)
        self.depth_color_map = cv2.applyColorMap(cv2.convertScaleAbs(self.depth, alpha=0.08), cv2.COLORMAP_JET)
        self.camera_depth = True

    def rs_callback(self, data):
        self.rgb_image = self.bridge.compressed_imgmsg_to_cv2(data)
        self.camera_RGB = True

    def YOLOv7_detect(self):
        img = self.rgb_image
        im0 = img.copy()
        img = img[np.newaxis, :, :, :]
        img = np.stack(img, 0)
        img = img[..., ::-1].transpose((0, 3, 1, 2))
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()
        img /= 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        if self.device.type != 'cpu' and (self.old_img_b != img.shape[0] or self.old_img_h != img.shape[2] or self.old_img_w != img.shape[3]):
            self.old_img_b = img.shape[0]
            self.old_img_h = img.shape[2]
            self.old_img_w = img.shape[3]
            for i in range(3):
                self.model(img)[0]

        t1 = time_synchronized()
        with torch.no_grad():
            pred = self.model(img)[0]
        t2 = time_synchronized()

        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)
        t3 = time_synchronized()

        for i, det in enumerate(pred):
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()

                for *xyxy, conf, cls in reversed(det):
                    label = f'{self.names[int(cls)]} {conf:.2f}'

                    if conf > 0.8:
                        plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=2)
                        plot_one_box(xyxy, self.depth_color_map, label=label, color=self.colors[int(cls)], line_thickness=2)
                        label_name = f'{self.names[int(cls)]}'
                        c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                        x = int((c2[0] + c1[0]) / 2)
                        y = int((c2[1] + c1[1]) / 2)

                        if x < 480 and y < 640 and self.depth[x][y] < 5000:
                            if self.intr:
                                real_coords = rs.rs2_deproject_pixel_to_point(self.intr, [x, y], self.depth[x][y])

                            if real_coords != [0.0, 0.0, 0.0]:
                                depth_scale = 0.001
                                self.get_logger().info(f"depth_coord = {real_coords[0] * depth_scale}  {real_coords[1] * depth_scale}  {real_coords[2] * depth_scale}")

            cv2.imshow("YOLOv7 Result RGB", im0)
            cv2.imshow("YOLOv7 Result Depth", self.depth_color_map)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def timer_callback(self):
        if self.camera_RGB == True:
            self.YOLOv7_detect()

def main(args=None):
    rclpy.init(args=args)
    with torch.no_grad():
        node = ObjectDetection()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
