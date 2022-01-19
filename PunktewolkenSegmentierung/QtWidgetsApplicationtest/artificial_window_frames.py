import h5py
import math
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation

# basic forms like cuboids, arcs, surfaces (individual filling with uniform dist. or overlay with cuboid)
# composition of basic forms to complete models (each part appends its own points)
# subsampling to specific number of points
# compress all models to h5 file
# train model and test on real data of same type

pointcloud_num = 1024
pointcloud_point_num = 2048

frame_width_min = 500
frame_width_max = 3000
frame_height_min = 500
frame_height_max = 3000
strut_width = 100
sigma = 30
clip = 50
a_class = np.full((pointcloud_num, 1), 0, dtype=np.single)


def create_window_frame(pointcloud_idx):
    a_data = np.zeros((pointcloud_point_num, 3), dtype=np.single)
    a_label = np.zeros((pointcloud_point_num, 2), dtype=np.single)

    print('Window frame index: ', pointcloud_idx)
    frame_width = np.random.uniform(frame_width_min, frame_width_max)
    frame_height = np.random.uniform(frame_height_min, frame_height_max)

    corner_point_num = int(pointcloud_point_num / pow(2, 6))
    strut_point_num = int(pointcloud_point_num / 4 - corner_point_num)

    # down left corner
    idx = 0
    a_data[idx:idx + corner_point_num] = np.random.uniform(
        [-frame_width / 2 - strut_width, -frame_height / 2 - strut_width, -strut_width / 2],
        [-frame_width / 2, -frame_height / 2, strut_width / 2], (corner_point_num, 3))
    # down right corner
    idx += corner_point_num
    a_data[idx:idx + corner_point_num] = np.random.uniform(
        [frame_width / 2, -frame_height / 2 - strut_width, -strut_width / 2],
        [frame_width / 2 + strut_width, -frame_height / 2, strut_width / 2], (corner_point_num, 3))
    # up left corner
    idx += corner_point_num
    a_data[idx:idx + corner_point_num] = np.random.uniform(
        [-frame_width / 2 - strut_width, frame_height / 2, -strut_width / 2],
        [-frame_width / 2, frame_height / 2 + strut_width, strut_width / 2], (corner_point_num, 3))
    # up right corner
    idx += corner_point_num
    a_data[idx:idx + corner_point_num] = np.random.uniform(
        [frame_width / 2, frame_height / 2, -strut_width / 2],
        [frame_width / 2 + strut_width, frame_height / 2 + strut_width, strut_width / 2], (corner_point_num, 3))
    idx += corner_point_num
    a_label[:idx, 0] = 1.0

    a_label[idx:, 1] = 1.0
    # lower strut
    a_data[idx:idx + math.ceil(strut_point_num * 2*frame_width/(frame_width+frame_height))] = np.random.uniform(
        [-frame_width / 2, -frame_height / 2 - strut_width, -strut_width / 2],
        [frame_width / 2, -frame_height / 2, strut_width / 2], (math.ceil(strut_point_num * 2*frame_width/(frame_width+frame_height)), 3))
    # left strut
    idx += math.ceil(strut_point_num * 2*frame_width/(frame_width+frame_height))
    a_data[idx:idx + math.floor(strut_point_num * 2*frame_height/(frame_width+frame_height))] = np.random.uniform(
        [-frame_width / 2 - strut_width, -frame_height / 2, -strut_width / 2],
        [-frame_width / 2, frame_height / 2, strut_width / 2], (math.floor(strut_point_num * 2*frame_height/(frame_width+frame_height)), 3))
    # upper strut
    idx += math.floor(strut_point_num * 2*frame_height/(frame_width+frame_height))
    a_data[idx:idx + math.ceil(strut_point_num * 2*frame_width/(frame_width+frame_height))] = np.random.uniform(
        [-frame_width / 2, frame_height / 2, -strut_width / 2],
        [frame_width / 2, frame_height / 2 + strut_width, strut_width / 2], (math.ceil(strut_point_num * 2*frame_width/(frame_width+frame_height)), 3))
    # right strut
    idx += math.ceil(strut_point_num * 2*frame_width/(frame_width+frame_height))
    a_data[idx:idx + math.floor(strut_point_num * 2*frame_height/(frame_width+frame_height))] = np.random.uniform(
        [frame_width / 2, -frame_height / 2, -strut_width / 2],
        [frame_width / 2 + strut_width, frame_height / 2, strut_width / 2], (math.floor(strut_point_num * 2*frame_height/(frame_width+frame_height)), 3))
    idx += math.floor(strut_point_num * 2*frame_height / (frame_width + frame_height))

    a_data += np.clip(sigma * np.random.randn(pointcloud_point_num, 3), -1 * clip, clip)
    # permute points
    perm = np.random.permutation(pointcloud_point_num)
    a_data = a_data[perm]
    a_label = a_label[perm]

    rotMat = Rotation.random()
    a_data = np.dot(a_data, rotMat.as_matrix())

    # fig = pyplot.figure()
    # ax = Axes3D(fig)
    # ax.scatter(a_data[:4 * corner_point_num, 0], a_data[:4 * corner_point_num, 1],
    #            a_data[:4 * corner_point_num, 2], c='r')
    # ax.scatter(a_data[4 * corner_point_num:, 0], a_data[4 * corner_point_num:, 1],
    #            a_data[4 * corner_point_num:, 2], c='g')
    # pyplot.show()

    return a_data, a_label


a_data_global = np.zeros((pointcloud_num, pointcloud_point_num, 3), dtype=np.single)
a_label_global = np.zeros((pointcloud_num, pointcloud_point_num, 2), dtype=np.single)
for i in range(pointcloud_num):
    a_data, a_label = create_window_frame(i)
    a_data_global[i] = a_data
    a_label_global[i] = a_label

hdf5_file = h5py.File("./Seg_Prep/training_data.h5", 'w')
hdf5_file.create_dataset("data", data=a_data_global, compression="gzip", compression_opts=9)
hdf5_file.create_dataset("label", data=a_label_global, compression="gzip", compression_opts=9)
hdf5_file.create_dataset("class", data=a_class, compression="gzip", compression_opts=9)
hdf5_file.close()
