#!/usr/bin/env python3


import rclpy
import cv2
import time
import random
import argparse
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

mostraPoly = ["rural_road", "road", "vegetation"]

path = "/home/rota_2024/nav2_gps_ws/src/navigation2/"
timestr = time.strftime("%Y%m%d_%H%M%S")

def load_classes():
    rede = "dnn_model/data.yaml"
    with open(path + rede, 'r') as config_file:
        config_data = config_file.read()

    class_names_start = config_data.find("names: [") + len("names: [")
    class_names_end = config_data.find("]", class_names_start)
    class_names_str = config_data[class_names_start:class_names_end]
    class_names = [name.strip().strip("'") for name in class_names_str.split(",")]

    return class_names

def class_colors(names):
    return {name: (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for name in names}

class segmentationPub(Node):
    def __init__(self, show, raw, infer, flir):
        super().__init__('image_segmentation')

        self.raw = raw
        self.infer = infer
        self.show = show
        self.flir = flir

        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)

        if raw:
            self.raw_publisher = self.create_publisher(Image, 'image_raw', 10)

        if infer:
            self.infer_publisher = self.create_publisher(Image, 'image_bb', 10)

        self.segmentation_publisher = self.create_publisher(Int8MultiArray, 'segmentation_mask', 10)

        self.model = YOLO(path + "dnn_model/best.pt")
        self.classes = load_classes()
        self.color = class_colors(self.classes)

        self.br = CvBridge()
        self.confidence_threshold = 0.8
        self.last_update_time = time.time()

    def listener_callback(self, data):
        current_time = time.time()
        if current_time - self.last_update_time < 1:  # Atualiza a cada segundo
            return

        self.last_update_time = current_time

        self.get_logger().info('Receiving video frame')
        frame = self.br.imgmsg_to_cv2(data)

        
        size = (data.width, data.height)
        print("Tamanho do Mapa de Ocuoação", size)

        try:
            results = self.model(frame)
            result = results[0]

            segmentation_contours_idx = []
            for seg in result.masks.xyn:
                seg[:, 0] *= size[0]
                seg[:, 1] *= size[1]
                segment = np.array(seg, dtype=np.int32)
                segmentation_contours_idx.append(segment)

            bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
            yoloClasses = np.array(result.boxes.cls.cpu(), dtype="int")
            scores = np.array(result.boxes.conf.cpu())

            mask = np.zeros((data.height, data.width), dtype=np.uint8)

            for bbox, class_id, seg, score in zip(bboxes, yoloClasses, segmentation_contours_idx, scores):
                className = self.classes[class_id]

                cv2.polylines(frame, [seg], True, self.color[className], 3)

                if className == "estradaTerra" or className =="estradaAsfalto":
                    cv2.fillPoly(mask, [seg], 0)  # Preenche o polígono da classe "estradaAsfalto" com 1
                    self.get_logger().info("Detected 'estrada', painting it as free (white)")
                else:
                    cv2.fillPoly(mask, [seg], 1)  # Preenche as outras classes com 0 (livre)
                    self.get_logger().info(f"Detected '{className}', painting it as occupied (black)")

            segmentation_msg = Int8MultiArray()
            segmentation_msg.layout.dim.append(MultiArrayDimension(label='height', size=mask.shape[0], stride=mask.shape[0] * mask.shape[1]))
            segmentation_msg.layout.dim.append(MultiArrayDimension(label='width', size=mask.shape[1], stride=mask.shape[1]))
            segmentation_msg.data = mask.flatten().tolist()
            self.segmentation_publisher.publish(segmentation_msg)

            if self.show:
                frame = cv2.resize(frame, None, fx=0.6, fy=0.6)
                cv2.imshow("Image", frame)
                cv2.waitKey(1) & 0xff

            if self.infer:
                self.infer_publisher.publish(self.br.cv2_to_imgmsg(frame))

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    print("ok")
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--show', type=int, default=1, help='Set to 1 to display images')
    parser.add_argument('--raw', type=int, default=0, help='Set to 1 to enable raw image publishing')
    parser.add_argument('--infer', type=int, default=1, help='Set to 1 to enable inferenced image publishing')
    parser.add_argument('--flir', type=int, default=1, help='Set to 1 to enable flir camera')

    args = parser.parse_args()

    image_publisher = segmentationPub(args.show, args.raw, args.infer, args.flir)

    try:
        while rclpy.ok():
            rclpy.spin_once(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()