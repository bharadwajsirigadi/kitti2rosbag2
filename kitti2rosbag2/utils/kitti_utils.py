import time
import os
import numpy as np
from pathlib import Path
import cv2

DATA_EXTENSION = ".png"
DATASET_DIR = "/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset" # Fix this line
ODOM_DIR = '/media/psf/SSD/DRONES_LAB/kitti_dataset/dataset_2'  # Fix this line
SEQUENCE = 0
LEFT_IMG_FOLDER = "image_0"
RIGHT_IMG_FOLDER = "image_1"
# DISTANCE = 0.54meters

class KITTIOdometryDataset():
    def __init__(self, data_dir, sequence: int, odom_dir = None,  *_, **__) -> None:
        self.kitti_sequence_dir = os.path.join(data_dir, "sequences", f'{sequence:02d}')
        if odom_dir is not None:
            self.odom_dir = os.path.join(odom_dir, 'poses', f'{sequence:02d}.txt')
        self.left_cam_sequence_dir = os.path.join(self.kitti_sequence_dir, LEFT_IMG_FOLDER)
        self.right_cam_sequence_dir = os.path.join(self.kitti_sequence_dir, RIGHT_IMG_FOLDER)
        self.calib_file = os.path.join(self.kitti_sequence_dir,"calib.txt")
        self.time_file = os.path.join(self.kitti_sequence_dir,"times.txt")

    def get_files(self, extension, dir:Path):
        files = os.listdir(dir)
        filtered_files = []
        for file in files:
            if file.endswith(extension):
                filtered_files.append(file)
        filtered_files.sort()
        return filtered_files
    
    def write_text(self, files_list, file_name):
        file_name = f"file_{file_name}.txt"
        file_path = os.path.join(self.sequence_dir, file_name)
        try:
            with open(file_path, 'w') as file:
                for item in files_list:
                    file.write(f"{item}\n")
            print(f"File '{file_name}' has been created and written to '{file_path}'.")
        except Exception as e:
            print(f"An error occurred: {e}")
        return
    
    def left_images(self):
        image_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".gif"]
        image_files = []
        for filename in os.listdir(self.left_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                image_files.append(os.path.join(self.left_cam_sequence_dir, filename))
        image_files = sorted(image_files)
        return image_files
    
    def right_images(self):
        image_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".gif"]
        image_files = []
        for filename in os.listdir(self.right_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                image_files.append(os.path.join(self.right_cam_sequence_dir, filename))
        image_files = sorted(image_files)
        return image_files
    
    def stereo_images(self):
        image_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".gif"]
        left_image_files = []
        for filename in os.listdir(self.left_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                left_image_files.append(os.path.join(self.left_cam_sequence_dir, filename))
        left_image_files = sorted(left_image_files)
        right_image_files = []
        for filename in os.listdir(self.right_cam_sequence_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                right_image_files.append(os.path.join(self.right_cam_sequence_dir, filename))
        right_image_files = sorted(right_image_files)
        print(len(right_image_files))
        return left_image_files, right_image_files
    
    def continuous_image_reader(self, images):
        if images == "right_images":
            image_files = self.right_images()
        elif images == "left_images":
            image_files = self.left_images()
        else:
            left_image_files, right_img_files = self.stereo_images()
        while True:
            if images == "right_images" or images == "left_images":
                for image_file in image_files:
                    image = cv2.imread(image_file)
                    window_name = "image display"
                    if image is not None:
                        print(f"Reading image: {image_file}")
                        cv2.imshow(window_name, image)
                        cv2.waitKey(55)
                time.sleep(5)
            else:
                for left_img_file, right_img_file in zip(left_image_files, right_img_files):
                    left_image = cv2.imread(left_img_file)
                    right_image = cv2.imread(right_img_file)
                    window_name = "Stereo display"
                    if left_image is not None and right_image is not None:
                        stereo_image = cv2.hconcat([left_image, right_image])
                        cv2.imshow("Stereo Display", stereo_image)
                        cv2.waitKey(55)
                    else:
                        print("One or both images are invalid.")
                time.sleep(5)
                cv2.destroyAllWindows()
                
    def projection_matrix(self, cam):
        projection_matrices = []
        with open(self.calib_file, 'r') as file:
            for line in file:
                if line.startswith('P'):
                    values = [float(x) for x in line.split(':')[1].strip().split()]
                    matrix = np.array(values).reshape(3, 4)
                    projection_matrices.append(matrix)
        return projection_matrices[cam]
    
    def times_file(self):
        matrix = []
        with open(self.time_file, 'r') as file:
            for line in file:
                matrix.append(float(line))
        return np.array(matrix)
    
    def odom_pose(self):
        if not os.path.exists(self.odom_dir):
            raise FileNotFoundError(f"Odom directory not found: {self.odom_dir}, Ground truth(Odometry) is available for only 10 sequences in KITTI. Stopping the process.")
        with open(self.odom_dir, 'r') as file:
            lines = file.readlines()
        transformation_data = [[float(val) for val in line.split()] for line in lines]
        homogenous_matrix_arr = []
        for i in range(len(transformation_data)):
            homogenous_matrix = np.identity(4)
            homogenous_matrix[0, :] = transformation_data[i][0:4]
            homogenous_matrix[1:2, :] = transformation_data[i][4:8]
            homogenous_matrix[2:3, :] = transformation_data[i][8:12]
            homogenous_matrix_arr.append(homogenous_matrix)
        return np.array(homogenous_matrix_arr)
    
    def __getitem__(self, idx):

        return

    def __len__(self):
        
        return

def main():
    kitti = KITTIOdometryDataset(DATASET_DIR, ODOM_DIR, SEQUENCE)
    matrix = kitti.projection_matrix(3)
    right, left = kitti.stereo_images()
    # kitti.times_file()
    # arr = kitti.odom_pose()
    return

if __name__=="__main__":
    main()
    
