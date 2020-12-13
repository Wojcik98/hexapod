import os
import platform

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = 'line_detection'


def main():
    pkg_path = get_package_share_directory(PACKAGE)
    rpi = platform.machine() == 'aarch64'
    camera = True

    if camera:
        if rpi:
            camera = cv2.VideoCapture(-1, cv2.CAP_V4L2)
        else:
            camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        _, img = camera.read()
        camera.release()
    else:
        img_path = '/src/line_detection/line_detection/calib.jpg'
        img = cv2.imread(img_path)

    plt.imsave('/home/ubuntu/orig.jpg', img)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    cont, hier = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cont, key=cv2.contourArea)
    # img_cont = np.zeros((480, 640), dtype=np.uint8)
    # cv2.drawContours(img_cont, [c], -1, 255, thickness=1)

    eps = 0.1 * cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, eps, True)
    print(approx)
    # img_simpler_shape = np.zeros((480, 640), dtype=np.uint8)
    # cv2.drawContours(img_simpler_shape, [approx], -1, 255, thickness=1)

    if approx.size != 8:
        raise Exception('Incorrect shape!')

    src = approx.reshape((4, 2)).astype('float32')
    src = order_points(src)
    dst = np.array([
        [0.150, -0.050],
        [0.150,  0.050],
        [0.250,  0.050],
        [0.250, -0.050]
    ], dtype='float32')

    M = cv2.getPerspectiveTransform(src, dst)

    transform_file_path = os.path.join(pkg_path, 'image_transform.yaml')
    with open(transform_file_path, 'w') as f:
        content = yaml.dump(M.tolist())
        f.write(content)

    print(f'Saved transformation to "{transform_file_path}"')

    # warped = cv2.warpPerspective(img, M, (640, 480))
    # cv2.imshow('yo', warped)
    # cv2.waitKey(1)


def order_points(points):
    rect = np.zeros((4, 2), dtype="float32")

    s = points.sum(axis=1)
    rect[0] = points[np.argmin(s)]
    rect[2] = points[np.argmax(s)]

    diff = np.diff(points, axis=1)
    rect[1] = points[np.argmin(diff)]
    rect[3] = points[np.argmax(diff)]

    return rect


if __name__ == '__main__':
    main()
