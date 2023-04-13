import numpy as np
import tf
from tf.transformations import quaternion_matrix

def se3_transform(self, p) -> list:
        
        t = [self.x_offset, self.y_offset, self.z_offset]
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw_offset)
        R = quaternion_matrix(q)[:3, :3]
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        p_transformed = tf.transformations.concatenate_matrices(T, [p[0], p[1], p[2], 1.0])
                
        return [p_transformed[0], p_transformed[1], p_transformed[2]]