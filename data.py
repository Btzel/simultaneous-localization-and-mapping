import numpy as np
import socket

def acquisition():
    with open(r'LidarData.txt', 'r') as file:
        lidar_dataset = file.read()

    dataset = lidar_dataset.split(';|\n')
    dataset = np.array(dataset)
    dataset = adaptation(dataset)
    
    dataset = conversion(dataset)
    return np.array(dataset)

def adaptation(dataset):
    preprocessed_dataset = []
    for i in dataset:
        points = i.replace(',', '.')
        points = points.split(';')
        points = [tuple(item.split(':')) for item in points]
        preprocessed_dataset.append(points)

    return np.array(preprocessed_dataset)

def conversion(dataset):
    cartesian_dataset = []
    for i in dataset:
        cartesian_scan = []
        for j in i:
            angle = float(j[0])
            distance = float(j[1])
            angle_rad = np.deg2rad(angle)
            x = distance * np.cos(angle_rad)
            y = distance * np.sin(angle_rad)
            cartesian_scan.append([x, y])
        cartesian_dataset.append(cartesian_scan)
    return np.array(cartesian_dataset)