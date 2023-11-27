import math
import numpy as np


class Quaternion():
    def __init__(self) -> None:
        # transformation_matrix = 4*4(homogenous transformation matrix)
        # self.rotation_matrix = transformation_matrix[:3, :3]
        # self.translation_matrix = transformation_matrix[:3, 3]
        pass

    def rotationmtx_to_quaternion(self, m):
        t = np.matrix.trace(m)
        q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        if(t > 0):
            t = np.sqrt(t + 1)
            q[0] = 0.5 * t
            t = 0.5/t
            q[1] = (m[2,1] - m[1,2]) * t
            q[2] = (m[0,2] - m[2,0]) * t
            q[3] = (m[1,0] - m[0,1]) * t

        else:
            i = 0
            if (m[1,1] > m[0,0]):
                i = 1
            if (m[2,2] > m[i,i]):
                i = 2
            j = (i+1)%3
            k = (j+1)%3

            t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[0] = (m[k,j] - m[j,k]) * t
            q[j] = (m[j,i] + m[i,j]) * t
            q[k] = (m[k,i] + m[i,k]) * t

        return q

    def transformation_to_quaternion(self, transformation_matrix):
        rotation_matrix = transformation_matrix[:3, :3]
        translation_matrix = transformation_matrix[:3, 3]
        # r11 = self.rotation_matrix[0,0]
        # r12 = self.rotation_matrix[0,1]
        # r13 = self.rotation_matrix[0,2]
        # r21 = self.rotation_matrix[1,0]
        # r22 = self.rotation_matrix[1,1]
        # r23 = self.rotation_matrix[1,2]
        # r31 = self.rotation_matrix[2,0]
        # r32 = self.rotation_matrix[2,1]
        # r33 = self.rotation_matrix[2,2]

        # q_0n = ((1+r11+r22+r33)/4)
        # q_1n = ((1+r11-r22-r33)/4)
        # q_2n = ((1-r11+r22-r33)/4)
        # q_3n = ((1-r11-r22+r33)/4)

        # if q_0n > 0:
        #     q_0 = abs(math.sqrt((1+r11+r22+r33)/4))
        #     q_0k = math.sqrt((1+r11+r22+r33)/4)
        # else:
        #     q_0 = abs(math.sqrt(-(1+r11+r22+r33)/4))
        #     q_0k = -(math.sqrt(-(1+r11+r22+r33)/4))
        
        # if q_1n > 0:
        #     q_1 = abs(math.sqrt((1+r11-r22-r33)/4))
        #     q_1k = q_1
        # else:
        #     q_1 = abs(math.sqrt(-(1+r11-r22-r33)/4))
        #     q_1k = -(math.sqrt(-(1+r11-r22-r33)/4))
        
        # if q_2n > 0:
        #     q_2 = abs(math.sqrt((1-r11+r22-r33)/4))
        #     q_2k = q_2
        # else:
        #     q_2 = abs(math.sqrt(-(1-r11+r22-r33)/4))
        #     q_2k = -(math.sqrt(-(1-r11+r22-r33)/4))
        
        # if q_3n > 0:
        #     q_3 = abs(math.sqrt((1-r11-r22+r33)/4))
        #     q_3k = q_3
        # else:
        #     q_3 = abs(math.sqrt(-(1-r11-r22+r33)/4))
        #     q_3k = -(math.sqrt(-(1-r11-r22+r33)/4))
            
        # q_lst = [q_0, q_1, q_2, q_3]
        # q_lst1 = [q_0k, q_1k, q_2k, q_3k]
        # max_key, max_value = self.max(q_list=q_lst)

        # if max_value == q_lst[0]:
        #     q0 = q_lst[0]
        #     q1 = ((r32-r23)/(4*q_lst[0]))
        #     q2 = ((r13-r31)/(4*q_lst[0]))
        #     q3 = ((r21-r12)/(4*q_lst[0]))
        # elif max_value == q_lst[1]:
        #     q0 = ((r32-r23)/(4*q_lst[1]))
        #     q1 = q_lst[1]
        #     q2 = ((r12+r21)/(4*q_lst[1]))
        #     q3 = ((r13+r31)/(4*q_lst[1]))
        # elif max_value == q_lst[2]:
        #     q0 = ((r13-r31)/(4*q_lst[2]))
        #     q1 = ((r12+r21)/(4*q_lst[2]))
        #     q2 = q_lst[1]
        #     q3 = ((r23+r32)/(4*q_lst[2]))
        # elif max_value == q_lst[3]:
        #     q0 = ((r21-r12)/(4*q_lst[3]))
        #     q1 = ((r13+r31)/(4*q_lst[3]))
        #     q2 = ((r23+r32)/(4*q_lst[3]))
        #     q3 = q_lst[3]
        # return q0, q1, q2, q3
    
        r11, r12, r13 = rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2]
        r21, r22, r23 = rotation_matrix[1, 0], rotation_matrix[1, 1], rotation_matrix[1, 2]
        r31, r32, r33 = rotation_matrix[2, 0], rotation_matrix[2, 1], rotation_matrix[2, 2]

        q_0n = ((1 + r11 + r22 + r33) / 4)
        q_1n = ((1 - r11 + r22 - r33) / 4)
        q_2n = ((1 - r11 - r22 + r33) / 4)
        q_3n = ((1 + r11 - r22 - r33) / 4)

        q_0 = math.sqrt(abs(q_0n)) * (1 if q_0n > 0 else -1)
        q_1 = math.sqrt(abs(q_1n)) * (1 if q_1n > 0 else -1)
        q_2 = math.sqrt(abs(q_2n)) * (1 if q_2n > 0 else -1)
        q_3 = math.sqrt(abs(q_3n)) * (1 if q_3n > 0 else -1)

        max_value = max(q_0, q_1, q_2, q_3)

        if max_value == q_0:
            q0 = max_value
            q1 = (r32 - r23) / (4 * max_value)
            q2 = (r13 - r31) / (4 * max_value)
            q3 = (r21 - r12) / (4 * max_value)
        elif max_value == q_1:
            q0 = (r32 - r23) / (4 * max_value)
            q1 = max_value
            q2 = (r12 + r21) / (4 * max_value)
            q3 = (r13 + r31) / (4 * max_value)
        elif max_value == q_2:
            q0 = (r13 - r31) / (4 * max_value)
            q1 = (r12 + r21) / (4 * max_value)
            q2 = max_value
            q3 = (r23 + r32) / (4 * max_value)
        elif max_value == q_3:
            q0 = (r21 - r12) / (4 * max_value)
            q1 = (r13 + r31) / (4 * max_value)
            q2 = (r23 + r32) / (4 * max_value)
            q3 = max_value

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
    q = Quaternion()
    print(q.transformation_to_quaternion(transformation_matrix))
    return

if __name__ == "__main__":
    main()

        