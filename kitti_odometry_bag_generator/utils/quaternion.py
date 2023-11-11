import math
import numpy as np


class Quaternion():
    def __init__(self, transformation_matrix) -> None:
        # transformation_matrix = 4*4(homogenous transformation matrix)
        self.rotation_matrix = transformation_matrix[:3, :3]
        self.translation_matrix = transformation_matrix[:3, 3]
        pass

    def transformation_to_quaternion(self):
        r11 = self.rotation_matrix[0,0]
        r12 = self.rotation_matrix[0,1]
        r13 = self.rotation_matrix[0,2]
        r21 = self.rotation_matrix[1,0]
        r22 = self.rotation_matrix[1,1]
        r23 = self.rotation_matrix[1,2]
        r31 = self.rotation_matrix[2,0]
        r32 = self.rotation_matrix[2,1]
        r33 = self.rotation_matrix[2,2]

        q_0n = ((1+r11+r22+r33)/4)
        q_1n = ((1+r11-r22-r33)/4)
        q_2n = ((1-r11+r22-r33)/4)
        q_3n = ((1-r11-r22+r33)/4)

        if q_0n > 0:
            q_0 = abs(math.sqrt((1+r11+r22+r33)/4))
        else:
            q_0 = abs(math.sqrt(-(1+r11+r22+r33)/4))
        
        if q_1n > 0:
            q_1 = abs(math.sqrt((1+r11-r22-r33)/4))
        else:
            q_1 = abs(math.sqrt(-(1+r11-r22-r33)/4))
        
        if q_2n > 0:
            q_2 = abs(math.sqrt((1-r11+r22-r33)/4))
        else:
            q_2 = abs(math.sqrt(-(1-r11+r22-r33)/4))
        
        if q_3n > 0:
            q_3 = abs(math.sqrt((1-r11-r22+r33)/4))
        else:
            q_3 = abs(math.sqrt(-(1-r11-r22+r33)/4))
            
        q_lst = [q_0, q_1, q_2, q_3]
        max_key, max_value = self.max(q_list=q_lst)

        if max_value == q_lst[0]:
            q0 = q_lst[0]
            q1 = ((r32-r23)/(4*q_lst[0]))
            q2 = ((r13-r31)/(4*q_lst[0]))
            q3 = ((r21-r12)/(4*q_lst[0]))
        elif max_value == q_lst[1]:
            q0 = ((r32-r23)/(4*q_lst[1]))
            q1 = q_lst[1]
            q2 = ((r12+r21)/(4*q_lst[1]))
            q3 = ((r13+r31)/(4*q_lst[1]))
        elif max_value == q_lst[2]:
            q0 = ((r13-r31)/(4*q_lst[2]))
            q1 = ((r12+r21)/(4*q_lst[2]))
            q2 = q_lst[1]
            q3 = ((r23+r32)/(4*q_lst[2]))
        elif max_value == q_lst[3]:
            q0 = ((r21-r12)/(4*q_lst[3]))
            q1 = ((r13+r31)/(4*q_lst[3]))
            q2 = ((r23+r32)/(4*q_lst[3]))
            q3 = q_lst[3]
        return q0, q1, q2, q3
    
    def max(self, q_list:list):
        values = dict()
        for i, q_val in enumerate(q_list):
            values[f'q_{i}'] = q_val
        max_key = max(values, key=values.get)
        max_value = values[max_key]
        return max_key, max_value

def main():
    transformation_matrix = np.array([[float(1.000000e+00), float(9.043680e-12), float(2.326809e-11), float(5.551115e-17)],
                                     [float(9.043683e-12),float(1.000000e+00), float(2.392370e-10), float(3.330669e-16)],
                                     [float(2.326810e-11), float(2.392370e-10),float(9.999999e-01), float(-4.440892e-16)],
                                     [0, 0, 0, 1]])
    print(transformation_matrix)
    q = Quaternion(transformation_matrix)
    print(q.transformation_to_quaternion())
    return

if __name__ == "__main__":
    main()

        