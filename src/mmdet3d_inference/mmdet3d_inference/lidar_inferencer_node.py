import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp
import numpy as np
from mmdet3d.apis import init_model, inference_detector
from object_detection_msgs.msg import Object3d, Object3dArray
from geometry_msgs.msg import Point
from numpy.lib.recfunctions import unstructured_to_structured

CLASS2LABEL = {
    'Pedestrian': 0, 
    'Cyclist': 1, 
    'Car': 2        
}


class MMDet3DLidarInferencerNode(Node):
    
    def __init__(self):
        super().__init__('mmdet3d_lidar_inferencer_node')

        # declare parameters
        self.declare_parameter("config_path", "")
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("score_threshold", 0.5)

        # get parameters
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        checkpoint_path = self.get_parameter("checkpoint_path").get_parameter_value().string_value
        device = self.get_parameter("device").get_parameter_value().string_value
        self.score_threshold = self.get_parameter("score_threshold").get_parameter_value().double_value

        self.model = init_model(config=config_path, checkpoint=checkpoint_path, device=device)

        self.lidar_subscription = self.create_subscription(
            msg_type = PointCloud2,
            topic = 'lidar',
            callback = self.lidar_callback, 
            qos_profile = 10
        )

        self.objects_publisher = self.create_publisher(Object3dArray, 'object_detections_3d', 10)
        self.lidar_publisher = self.create_publisher(PointCloud2, 'lidar', 10)

        self.create_timer(
            timer_period_sec=1.0,
            callback=self.timer_cb
        )


    def lidar_callback(self, lidar_msg: PointCloud2):
        pcd = rnp.numpify(lidar_msg)
        pcd = np.array([ pcd['x'].flatten(), pcd['y'].flatten(), pcd['z'].flatten(), pcd['reflectivity'].flatten()]).T
        pcd[:, 3] /= 255.0

        results, _ = inference_detector(model=self.model, pcds=pcd)
        bbox_corners = results.pred_instances_3d.bboxes_3d.corners
        labels = results.pred_instances_3d.labels_3d
        scores = results.pred_instances_3d.scores_3d

        if len(bbox_corners) == 0:
            return

        object_instance_array = Object3dArray()
        for corners, label, score in zip(bbox_corners, labels, scores):
            if score < self.score_threshold:
                continue

            object_instance = Object3d()
            (x0y0z0, x0y0z1, x0y1z1, x0y1z0, x1y0z0, x1y0z1, x1y1z1, x1y1z0) = corners
            object_instance.bounding_box.corners[3] = self.convert_to_point_msg(x0y0z0)
            object_instance.bounding_box.corners[2] = self.convert_to_point_msg(x0y0z1)
            object_instance.bounding_box.corners[1] = self.convert_to_point_msg(x0y1z1)
            object_instance.bounding_box.corners[0] = self.convert_to_point_msg(x0y1z0)
            object_instance.bounding_box.corners[7] = self.convert_to_point_msg(x1y0z0)
            object_instance.bounding_box.corners[6] = self.convert_to_point_msg(x1y0z1)
            object_instance.bounding_box.corners[5] = self.convert_to_point_msg(x1y1z1)
            object_instance.bounding_box.corners[4] = self.convert_to_point_msg(x1y1z0)

            object_instance.label = CLASS2LABEL[self.model.dataset_meta['classes'][label]]
            object_instance.confidence_score = float(score)
            object_instance_array.objects.append(object_instance)

        
        object_instance_array.header.stamp = self.get_clock().now().to_msg()
        object_instance_array.header.frame_id = lidar_msg.header.frame_id
        self.objects_publisher.publish(object_instance_array)
        
    def convert_to_point_msg(self, point):
        point_msg = Point()
        point_msg.x = float(point[0])
        point_msg.y = float(point[1])
        point_msg.z = float(point[2])
        return point_msg

        

        

def main(args=None):
    rclpy.init(args=args)

    lidar_inferencer_node = MMDet3DLidarInferencerNode()
    rclpy.spin(lidar_inferencer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_inferencer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()