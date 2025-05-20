# Demonstrating an Established Optical-Flow-Based Method for Mobile Robot Dynamic Obstacle Detection 

## Purpose
TODO: Explain how this project is the result of a two week assignment for a  course on robot perception. 

## The Basic Premise
The approach is premised on the following: 
* The mathematical technique of dense optical flow provides an estimation of pixel motion between robot camera images taken closely together in time. The technique takes as input two such frames and produces as output a field of R2 vectors showing pixel translational motion. The pixel motion is caused by relative motion between the robot camera and all of the objects in the robot’s scene. 
* It has been shown in the literature that if we assume that this relative motion is caused only by the rotation and translation of the robot’s camera and that the robot’s camera scene is largely planar, then the optical flow field can be approximated by a single affine transformation (linear transformation plus translation) so long as we also assume that camera rotation between frames can only be nonzero about its yaw axis. 
* If we reproject all pixels from the first frame with the approximation affine transformation and compare their landing spots with the landing spots computed with a dense optical flow algorithm, then we reveal all segments of the image that deviate from our assumptions, providing a way of dynamic obstacle detection even when the robot itself is moving.

## The Mathematics that Underpin the Premise
The mathematics that explain the premise involves deriving a homographic transformation to describe how the pixel representation of a given planar scene changes between optical flow images taken closely together in time, then reducing this homographic transformation into an affine transformation leveraging that the robot camera moves between frames but that the camera rotation only involves yaw.

* We begin by lifting all pixels belonging to the planar scene into the image plane via the camera intrinsics.

$$ \mathbf{X}_{C1}' = K^{-1} \mathbf{x}_1 $$

* The result is a ray indicating 3D direction in the reference frame of the camera before the motion step. We multiply this ray by the constant $ Z_{C1} $ to indicate a 3D image point. 

$$ \mathbf{X}_{C1} = Z_{C1} \mathbf{X}_{C1}' = Z_{C1} K^{-1} \mathbf{x}_1 $$

* Employing the fact that all points being transformed belong to the same plane, $ \mathbf{n}^T \mathbf{X}_{C1} = d $, we draw the following sequence to eliminate the $Z_{C1}$ ambiguity. 

$$ \mathbf{n}_{C1}^T (Z_{C1} K^{-1} \mathbf{x}_1) = d $$

$$ Z_{C1} = \frac{d}{\mathbf{n}^T K^{-1} \mathbf{x}_1} $$

$$ \mathbf{X}_{C1} = \frac{d}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K^{-1} \mathbf{x}_1 $$

* With the $Z_{C1}$ ambiguity eliminated, we proceed to model the effect of the camera's between-frame rigid body motion to express the mapping from the original pixel coordinates to post-motion coordinates in the new camera reference frame.  

$$ \mathbf{X}_{C2} = \mathbf{R} \left( \frac{d}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K^{-1} \mathbf{x}_1 \right) + \mathbf{t} $$

* We then introduce the structure for the homography by passing $ \mathbf{X}_{C2} $ through the camera intrinsics, then perform a sequence of algebriac simplifactions to draw out then remove unecessary constants and express a proper homography.

$$ \mathbf{x}_2 \sim K \left( \mathbf{R} \left( \frac{d}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K^{-1} \mathbf{x}_1 \right) + \mathbf{t} \right) $$

$$ \mathbf{x}_2 \sim K \left( \mathbf{R} \left( \frac{d}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K^{-1} \mathbf{x}_1 \right) + \mathbf{t} \left( \frac{\mathbf{n}^T K^{-1} \mathbf{x}_1}{\mathbf{n}^T K^{-1} \mathbf{x}_1} \right) \right) $$

$$ \mathbf{x}_2 \sim \frac{1}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K \left( d \mathbf{R} K^{-1} \mathbf{x}_1 + \mathbf{t} (\mathbf{n}^T K^{-1} \mathbf{x}_1) \right) $$

$$ \mathbf{x}_2 \sim \frac{1}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K \left( (d \mathbf{R}) K^{-1} \mathbf{x}_1 + (\mathbf{t}\mathbf{n}^T) K^{-1} \mathbf{x}_1 \right) $$

$$ \mathbf{x}_2 \sim \frac{1}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K \left( d \mathbf{R} + \mathbf{t}\mathbf{n}^T \right) K^{-1} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim \frac{1}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K \left( d \left( \mathbf{R} + \frac{\mathbf{t}\mathbf{n}^T}{d} \right) \right) K^{-1} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim \frac{d}{\mathbf{n}^T K^{-1} \mathbf{x}_1} K \left( \mathbf{R} + \frac{\mathbf{t}\mathbf{n}^T}{d} \right) K^{-1} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim K \left( \mathbf{R} + \frac{\mathbf{t}\mathbf{n}^T}{d} \right) K^{-1} \mathbf{x}_1 $$

* For the next step of the mathematical explanation of the premise, we recognize that the robot only has time to perform small rotations between successive image captures. This leads to a new expression for the homography, after expanding a general roll-pitch-yaw rotation matrix and deriving its first-order Taylor approximation. 

$$ \mathbf{R} = \mathbf{R}_x(\theta_x) \mathbf{R}_y(\theta_y) \mathbf{R}_z(\theta_z) $$

$$ \mathbf{R} = \begin{bmatrix} \cos \theta_y \cos \theta_z & -\cos \theta_y \sin \theta_z & \sin \theta_y \\ \cos \theta_x \sin \theta_z + \sin \theta_x \sin \theta_y \cos \theta_z & \cos \theta_x \cos \theta_z - \sin \theta_x \sin \theta_y \sin \theta_z & -\sin \theta_x \cos \theta_y \\ \sin \theta_x \sin \theta_z - \cos \theta_x \sin \theta_y \cos \theta_z & \sin \theta_x \cos \theta_z + \cos \theta_x \sin \theta_y \sin \theta_z & \cos \theta_x \cos \theta_y \end{bmatrix} $$

$$ \mathbf{R} \approx \begin{bmatrix} 1 & -\theta_z & \theta_y \\ \theta_z & 1 & -\theta_x \\ -\theta_y & \theta_x & 1 \end{bmatrix} $$

* Furthremore, roll and pitch are assumed to be zero and the perpendicular distance to the plane is safely assumed to be much larger than the z-axis translation that occurs between successive image captures. 

$$ \mathbf{R} \approx \begin{bmatrix} 1 & -\theta_z & 0 \\ \theta_z & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} $$

$$ \mathbf{x}_2 \sim K \left( \begin{bmatrix} 1 & -\theta_z & 0 \\ \theta_z & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} + \frac{\mathbf{t}\mathbf{n}^T}{d} \right) K^{-1} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim K \left( \begin{bmatrix} 1 & -\theta_z & 0 \\ \theta_z & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} + \frac{\mathbf{t}\mathbf{n}^T}{d} \right) K^{-1} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim K \left( \begin{bmatrix} 1 & -\theta_z & 0 \\ \theta_z & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} + \frac{1}{d} \begin{bmatrix} t_x n_x & t_x n_y & t_x n_z \\ t_y n_x & t_y n_y & t_y n_z \\ t_z n_x & t_z n_y & t_z n_z \end{bmatrix} \right) K^{-1} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim K \begin{bmatrix} 1 + \frac{t_x n_x}{d} & -\theta_z + \frac{t_x n_y}{d} & \frac{t_x n_z}{d} \\ \theta_z + \frac{t_y n_x}{d} & 1 + \frac{t_y n_y}{d} & \frac{t_y n_z}{d} \\ \frac{t_z n_x}{d} & \frac{t_z n_y}{d} & 1 + \frac{t_z n_z}{d} \end{bmatrix} K^{-1} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim K \begin{bmatrix} 1 + \frac{t_x n_x}{d} & -\theta_z + \frac{t_x n_y}{d} & \frac{t_x n_z}{d} \\ \theta_z + \frac{t_y n_x}{d} & 1 + \frac{t_y n_y}{d} & \frac{t_y n_z}{d} \\ 0 & 0 & 1 \end{bmatrix} K^{-1} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim \begin{bmatrix} f_x & s & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1 + \frac{t_x n_x}{d} & -\theta_z + \frac{t_x n_y}{d} & \frac{t_x n_z}{d} \\ \theta_z + \frac{t_y n_x}{d} & 1 + \frac{t_y n_y}{d} & \frac{t_y n_z}{d} \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1/f_x & -s/(f_x f_y) & (s c_y - f_y c_x)/(f_x f_y) \\ 0 & 1/f_y & -c_y/f_y \\ 0 & 0 & 1 \end{bmatrix} \mathbf{x}_1 $$

$$ \mathbf{x}_2 \sim \begin{bmatrix} A_1 & A_2 & A_3 \\ A_4 & A_5 & A_6 \\ 0 & 0 & 1 \end{bmatrix} \mathbf{x}_1 $$

With our approach, we develop a new affine transformation computationally at each image processing step, then we assert that all pixels should move according to this affine rule, with pixels deviating from this rule being indicative of relative motion between the robot camera and the capture scene that is not explained by the robot's own motion so thus must be belonging to a dynamic obstacle.

TODO: Show why an affine does well to approximate a robot's own ego motion when the above conditions hold. 

## The Implementation
The implementation involved the following:

* Standardizing frames obtained from the TurtleBot robot through a resizing operation to reduce computational complexity and a crop around an ROI operation to enforce the planar assumption. 
* Computing the optical flow between successive frames using OpenCV’s Farneback Dense Optical Flow framework (`cv2.calcOpticalFlowFarneback()`) with the default recommended settings.
* Developing an estimation affine 2x3 transformation matrix to model relative motion caused only by the robot’s own motion. This we accomplished with `cv2.estimateAffinePartial2D()` set with RANSAC.
* Developing a detection mask based on the reprojection error between the estimated affine transformation and the apparent motion computed with dense optical flow.
* Stopping the robot when the detection mask, smoothed temporally to maintain object constancy, fills beyond a static threshold. 

## The Results and The Challenges
We are realizing the following results and challenges:
* In repeated trials in simulation tool Gazebo, we have realized accurate dynamic obstacle detection with no false positives. This is demonstrated by the ROS2 node flagging the robot’s approach to the moving obstacle soda can as being `unsafe` but not flagging any other scenes as being `unsafe`. 
* The fact that our implementation relies on static thresholding for the RANSAC affine fitting step and for the final decision maker step means results will always be scene dependent and will always require parameter tuning. 
    * For example, we have found that the increasing the RANSAC inlier threshold reduces false positives at the sacrifice of not always detecting true dynamic obstacle motion. 
    * Moreover, we have realized that lowering the threshold requirement for mask inliers increases the frequency of false detections.

* Conceptually, the approach as a whole is premised on that dynamic obstacles never occupy the majority of pixels in any pair of successive frames. When this assumption fails to hold true, detections are meaningless.
    * Think of several dozen moving soda cans flooding the robot’s field of view. If this were to happen, the affine fit representing the robot’s own “ego” motion would too much be actually modeling the motion of dynamic obstacles.

### Result Case 1. Robot stationary and dynamic obstacle present.
![Result1](result2.png)

### Result Case 2. Robot moving and dynamic obstacle present
![Result2](result1.png)

### Result Case 3. Robot moving and dynamic obstacle present 
![Result3](result3.png)

## The Conclusion
* In conclusion, we have validated a well-established conceptual framework for dynamic obstacle detection–one that centers on operating on the direct comparison between computed, apparent dense optical flow and a RANSAC-fitted affine transformation summarizing the “gist” of this dense flow field. That being said, while our results have been intriguing, a truly robust, practical rendition–immune to changes in scene and free from guess and check hard-coded parameter settings–would require additional time researching and understanding more of what’s already been achieved in the literature and in industry. 

## References
1. https://oa.upm.es/21899/1/GONZALO_RUY_RODRIGUEZ_CANOSA.pdf   
2. https://www.r-5.org/files/books/computers/algo-list/image-processing/vision/Richard_Hartley_Andrew_Zisserman-Multiple_View_Geometry_in_Computer_Vision-EN.pdf
3. http://users.ece.northwestern.edu/~yingwu/teaching/EECS432/Notes/optical_flow.pdf
4. https://www.weizmann.ac.il/math/ronen/sites/math.ronen/files/uploads/basri_-_paraperspective_affine.pdf