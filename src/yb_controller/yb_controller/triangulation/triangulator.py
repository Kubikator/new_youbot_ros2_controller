import numpy as np
import cv2
from typing import Tuple, List, Optional

class Triangulation:
    def __init__(self, camera_matrix: np.ndarray):

        self.camera_matrix = camera_matrix
        self.dist_coeffs = np.zeros(5)  # предполагаем отсутствие дисторсии
        
    def triangulate_points(self,
                          pose1: Tuple[np.ndarray, np.ndarray],
                          pose2: Tuple[np.ndarray, np.ndarray],
                          points1: np.ndarray,
                          points2: np.ndarray) -> np.ndarray:

        R1, t1 = pose1
        R2, t2 = pose2
        
        # Создаем матрицы проекции
        P1 = self.camera_matrix @ np.hstack([R1, t1.reshape(-1, 1)])
        P2 = self.camera_matrix @ np.hstack([R2, t2.reshape(-1, 1)])
        
        # Триангуляция
        points4d = cv2.triangulatePoints(P1, P2, points1.T, points2.T)
        
        # Переводим из однородных координат в 3D
        points3d = cv2.convertPointsFromHomogeneous(points4d.T)
        return points3d.reshape(-1, 3)
    
    def triangulate_single_object(self,
                                 pose1: Tuple[np.ndarray, np.ndarray],
                                 pose2: Tuple[np.ndarray, np.ndarray],
                                 point1: np.ndarray,
                                 point2: np.ndarray) -> np.ndarray:
        
        points1 = point1.reshape(1, 2)
        points2 = point2.reshape(1, 2)
        
        points3d = self.triangulate_points(pose1, pose2, points1, points2)
        return points3d[0]
    
    def calculate_camera_poses(self,
                              R1: np.ndarray, t1: np.ndarray,
                              R2: np.ndarray, t2: np.ndarray) -> Tuple[Tuple, Tuple]:

        # Для триангуляции нужны матрицы проекции вида K[R|t]
        # где R, t - преобразование из мировой СК в систему координат камеры
        return (R1, t1), (R2, t2)
    