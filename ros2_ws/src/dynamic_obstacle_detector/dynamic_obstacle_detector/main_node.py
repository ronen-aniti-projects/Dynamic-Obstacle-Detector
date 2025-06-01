#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import cv2 
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist 
import numpy as np 

class DetectionNode(Node):
    def __init__(self):
        
        # -- ROS node setup
        super().__init__("detection_node")
        self.cv_bridge = CvBridge() 
        self.sub_img = self.create_subscription(
            Image, "/camera/image_raw", self.main_cb, 10)
        self.sub_teleop = self.create_subscription(
            Twist, '/input_cmd_vel', self.teleop_cb, 10)
        self.pub_detection = self.create_publisher(
            Bool, "/detection", 1)
        self.pub_flow_img = self.create_publisher(
            Image, '/flow_img', 10)
        self.pub_est_flow_img = self.create_publisher(
            Image, 'est_flow_img', 10)
        self.pub_res_img = self.create_publisher(
            Image, 'res_flow_img', 10)
        self.pub_mask_img = self.create_publisher(
            Image, '/mask_img', 10)
        self.pub_composite_display = self.create_publisher(
            Image, '/composite_display', 10) 
        self.pub_detection_bool = self.create_publisher(
            Bool, '/detection_bool', 10)
        self.pub_cmd_vel = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        # Dynamic obstacle detection state variables and settings
        self.detection = False
        self.frame1, self.frame2 = None, None
        self.flow_field = None
        self.flow_img = None
        self.modeled_flow_field = None
        self.modeled_flow_img = None
        self.residual_img = None   
        self.affine = None 
        self.mask = None 
        self.smoothed_mask = None 
        self.residual_img = None 
        self.ROI_width_low, self.ROI_width_high = 0.0, 1.0
        self.ROI_height_low, self.ROI_height_high = 0.5, 1.0
        self.standard_size = (250, 250)
        self.mask_filter_alpha = 0.8#0.25
        self.mask_filter_tau = 0.9
        self.mag_cap = 5.0
        self.ransac_thresh = 2.0#5.0#4.0#5.0#4.0
        self.gaussian_blur_size = (25, 25)
        self.outlier_percent_thresh = 5.0

    def convert_gray(self, bgr):
        """
        Return a GRAYSCALE frame given a BGR frame 
        """
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    
    def crop_roi(self, gray):
        """
        Return a GRAYSCALE frame, cropped to the pre-set ROI
        """
        H, W = gray.shape 
        y_low = int(self.ROI_height_low * H)
        y_high = int(self.ROI_height_high * H)
        x_low = int(self.ROI_width_low * W)
        x_high = int(self.ROI_width_high * W)
        return gray[y_low:y_high, x_low:x_high]
    
    def standardize(self, frame):
        """
        Return a GRAYSCALE frame, resized then cropped to the pre-set ROI
        """
        resized = cv2.resize(frame, self.standard_size, interpolation=cv2.INTER_AREA)
        return self.crop_roi(self.convert_gray(resized))
    
    def optical_flow(self):
        """
        Return the pixel FLOW FIELD (apparent motion) between two frames.
        """
        optical_flow = cv2.calcOpticalFlowFarneback(
            self.frame1, self.frame2, None,
            0.5, 5, 30, 5, 5, 1.2, 0)
        return cv2.GaussianBlur(optical_flow, self.gaussian_blur_size, 0) 

    def flow_image(self):
        """
        Return a BGR representation of the FLOW FIELD. 
        """
        mag, ang = cv2.cartToPolar(self.flow_field[..., 0], self.flow_field[..., 1])
        hsv = np.zeros((self.flow_field.shape[0], self.flow_field.shape[1], 3), dtype=np.uint8)
        hsv[..., 0] = (ang * 180 / np.pi / 2).astype(np.uint8) # map to [0,180]
        hsv[..., 1] = 255 
        hsv[..., 2] = mag / self.mag_cap * 255.0
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    
    def est_flow_image(self):
        """
        Return a BGR representation of the FLOW FIELD. 
        """
        mag, ang = cv2.cartToPolar(self.modeled_flow_field[..., 0], self.modeled_flow_field[..., 1])
        hsv = np.zeros((self.modeled_flow_field.shape[0], self.modeled_flow_field.shape[1], 3), dtype=np.uint8)
        hsv[..., 0] = (ang * 180 / np.pi / 2).astype(np.uint8) # map to [0,180]
        hsv[..., 1] = 255 
        hsv[..., 2] = mag / self.mag_cap * 255.0
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    

    def residual_flow_image(self):
        """
        Return a BGR representation of the RESIDUAL FLOW FIELD. 
        """
        mag, ang = cv2.cartToPolar(self.residual_flow_field[...,0],
                                   self.residual_flow_field[...,1])
        hsv = np.zeros((self.residual_flow_field.shape[0], self.residual_flow_field.shape[1], 3), dtype=np.uint8)
        hsv[..., 0] = (ang * 180 / np.pi / 2).astype(np.uint8) # map to [0,180]
        hsv[..., 1] = 255 
        hsv[..., 2] = mag / self.mag_cap * 255.0
        #hsv[~self.mask] = 0 # Zero the inliers
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    
    def estimate_affine_ransac(self):
        """
        Returns a RANSAC-fitted affine fit (and inlier mask) to the input flow field. 
        """
        H, W = self.frame1.shape 
        Y, X = np.indices((H, W))
        
        pre_image = np.stack((X.ravel(), Y.ravel()), axis=1).astype(np.float32)
        image = np.stack((X.ravel() + self.flow_field[...,0].ravel(), 
                         Y.ravel() + self.flow_field[...,1].ravel()), axis=1).astype(np.float32)
        affine, inliers = cv2.estimateAffine2D(
            pre_image, image, method=cv2.RANSAC, ransacReprojThreshold=self.ransac_thresh)
        
        #affine, inliers = cv2.estimateAffinePartial2D(
        #    pre_image, image, method=cv2.RANSAC, ransacReprojThreshold=self.ransac_thresh)
        return affine, inliers.reshape(H, W)

    def smooth_mask(self):
        """
        Return a SMOOTHED outlier mask given a raw mask and exponential moving average settings
        """
        mask_float = self.mask.astype(np.float32)
        if self.smoothed_mask is None: 
            self.smoothed_mask = mask_float.copy()
        else:
            self.smoothed_mask = self.mask_filter_alpha * mask_float + \
                (1 - self.mask_filter_alpha) * self.smoothed_mask
        return (self.smoothed_mask >= self.mask_filter_tau).astype(np.uint8) * 255

    def model_affine_flow(self):
        """
        Returns the MODELED FLOW FIELD under the AFFINE projection, given the affine. 
        """
        H, W = self.frame1.shape 
        Y, X = np.indices((H, W))
        pre_image = np.stack((X.ravel(), Y.ravel()), axis=1).astype(np.float32)
        projection = cv2.transform(pre_image.reshape(-1,1,2), self.affine).reshape(H, W, 2)
        return projection - np.stack((X, Y), axis=2)
    
    def compute_residual(self):
        """
        Returns the RESIDUAL FLOW FIELD (pixel motion not assumed ego)
        """
        return self.flow_field - self.modeled_flow_field

    def evaluate_mask(self):
        """
        Returns True if the smoothed outlier mask has at or above the threshold fraction
        of outliers, assumed to be explained by dynamic obstacle motion. Returns False
        otherwise. 
        """
        percent = 100 * (self.mask_smoothed / 255).mean()
        self.get_logger().info(f"Percent smoothed mask (outliers): {percent: .2f}%")
        return percent > self.outlier_percent_thresh
    
    def teleop_cb(self, msg: Twist):
        """
        Gates the teleop Twist message by the current detection flag. When a dynamic obstacle
        is detected, this callback will command zero velocity on topic /cmd_vel.
        """
        x = msg.linear.x 
        w = msg.angular.z 
        if self.detection: 
            out = Twist() 
            self.get_logger().warn("Overriding teleop because of detection")
        else: 
            out = msg 
        self.pub_cmd_vel.publish(out)

    def main_cb(self, msg: Image):
        
        try:
            # Convert the current image message to BGR
            cv_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Only proceed with calculations after aquiring two frames
            if self.frame1 is None:
                self.frame1 = self.standardize(cv_frame)
                return 
            self.frame2 = self.standardize(cv_frame)

            # Compute the apparent motion between two most recent frames
            self.flow_field = self.optical_flow()

            # Generate a BGR image of the flow field for visualization
            self.flow_img = self.flow_image()

            # Publish the flow field image:
            self.pub_flow_img.publish(
                self.cv_bridge.cv2_to_imgmsg(self.flow_img, encoding='bgr8'))

            # Model the ego-motion of the robot camera with a RANSAC-fitted affine
            self.affine, inliers = self.estimate_affine_ransac()
            if self.affine is None:
                self.frame1 = self.frame2.copy()
                return
            self.mask = (inliers == 0) # mark outliers
            self.modeled_flow_field = self.model_affine_flow()
            
            # Generate an estimated flow image
            self.modeled_flow_img = self.est_flow_image()

            # Publish the estimated flow field (ego-motion)
            self.pub_est_flow_img.publish(
                self.cv_bridge.cv2_to_imgmsg(self.modeled_flow_img, encoding='bgr8'))
            
            # Compute the residual flow field 
            self.residual_flow_field = self.compute_residual()

            # Generate a BGR image of the residual flow field for visualization
            self.residual_img = self.residual_flow_image()
            
            # Publish the residual image
            self.pub_res_img.publish(
                self.cv_bridge.cv2_to_imgmsg(self.residual_img, encoding='bgr8'))

            # Smooth the RANSAC outlier mask temporally
            self.mask_smoothed = self.smooth_mask()

            # Publish the smoothed outlier mask
            self.pub_mask_img.publish(
                self.cv_bridge.cv2_to_imgmsg(self.mask_smoothed, encoding='mono8'))
            
            # Determine if the scene is safe
            self.detection = self.evaluate_mask()
            detection_msg = Bool()
            detection_msg.data = bool(self.detection) 
            self.pub_detection_bool.publish(detection_msg)

            # Prepare base images for the composite display
            display_raw_cam_unlabeled = cv2.resize(cv_frame, self.standard_size, interpolation=cv2.INTER_AREA)
            mask_bgr_unlabeled = cv2.cvtColor(self.mask_smoothed, cv2.COLOR_GRAY2BGR)

            # Text properties
            font = cv2.FONT_HERSHEY_SIMPLEX
            color = (255,255,255); 
            line = 1
            org_s = (5,15)
            org_l = (5,20) 
            fs_s = 0.4
            fs_l = 0.5    

            img_raw_labeled = display_raw_cam_unlabeled.copy()
            cv2.putText(img_raw_labeled, 'Input Cam', org_l, font, fs_l, color, line)
            if self.detection:
                cv2.putText(img_raw_labeled, 'DYNAMIC OBSTACLE!', (int(0.1*img_raw_labeled.shape[1]), int(0.9*img_raw_labeled.shape[0])), 
                            font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            

            img_flow_labeled = self.flow_img.copy()
            cv2.putText(img_flow_labeled, 'Optical Flow', org_s, font, fs_s, color, line)
            
            img_model_labeled = self.modeled_flow_img.copy()
            cv2.putText(img_model_labeled, 'Ego-Motion', org_s, font, fs_s, color, line)
            
            img_resid_labeled = self.residual_img.copy()
            cv2.putText(img_resid_labeled, 'Residual', org_s, font, fs_s, color, line)
            
            img_mask_labeled = mask_bgr_unlabeled.copy()
            cv2.putText(img_mask_labeled, 'RANSAC Outliers', org_s, font, fs_s, color, line)
            
            bottom_panel_labeled = cv2.vconcat([img_flow_labeled, img_model_labeled, img_resid_labeled, img_mask_labeled])
            composite_display_labeled = cv2.vconcat([img_raw_labeled, bottom_panel_labeled])
            
            self.pub_composite_display.publish(
                self.cv_bridge.cv2_to_imgmsg(composite_display_labeled, encoding='bgr8'))

            # Perform a frame update
            self.frame1 = self.frame2.copy()

        except Exception as e:
            self.get_logger().error(f"Error in main_cb: {e}. Skipping this frame.")
            self.frame1, self.frame2 = None, None
            return
            
        

    


        

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode() 
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()








