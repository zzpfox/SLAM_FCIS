#!/usr/bin/env python
"""
Use Socket to receive color image and depth image from SLAM, and perform
Segmentation on the color image, calculate the segmented object's
point cloud, get center of mass(COM), send the object's type and COM back to SLAM

Author: Tao Chen
Email: chentao904@163.com
Date: 2017.05.18
"""
from __future__ import division
from __future__ import absolute_import
from __future__ import print_function

import argparse
import os
import sys
this_dir = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(this_dir, '..', '..', 'fcis'))
sys.path.insert(0, os.path.join(this_dir, '..', '..', 'lib'))

import pprint
import cv2
import random
from socketCom.Communication import Server
from socket import error as SocketError
import errno
from config.config import config, update_config
from utils.image import resize, transform
import numpy as np
# get config
os.environ['PYTHONUNBUFFERED'] = '1'
os.environ['MXNET_CUDNN_AUTOTUNE_DEFAULT'] = '0'
os.environ['MXNET_ENABLE_GPU_P2P'] = '0'
cur_path = os.path.abspath(os.path.dirname(__file__))
update_config(cur_path + '/cfgs/fcis_slxrobot_demo.yaml')


import mxnet as mx
import glob
import json
import time
print("use mxnet at", mx.__file__)
from core.tester import im_detect, Predictor
from symbols import *
from utils.load_model import load_param
from utils.show_masks import show_masks
from utils.tictoc import tic, toc
from nms.nms import py_nms_wrapper
from mask.mask_transform import gpu_mask_voting, cpu_mask_voting
import matplotlib.pyplot as plt

FX = 523.5967
FY = 524.3176
CX = 475.3411
CY = 257.3514

def show_mask(color, roimask, name, alpha=0.4):
    mask_row, mask_col = np.where(roimask > 0)
    color[mask_row, mask_col] = alpha * np.repeat(roimask[mask_row, mask_col, np.newaxis], 3, axis=-1)*255 \
                                + (1 - alpha) * color[mask_row, mask_col]
    plt.cla()
    plt.title('%s' % name)
    plt.axis("off")
    plt.imshow(color[:, :, -1::-1])
    plt.show()

def generate_point_cloud(depth, roimask, cod):
    """Transform a depth image data (CV_32F) into a point cloud with one point for each
    pixel in the image, using the camera transform for a camera
    centred at cx, cy with field of view fx, fy.

    The result is a 3-D array with
    shape (rows, cols, 3). Pixels with invalid depth in the input have
    NaN for the z-coordinate in the result.

    """
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    c += cod[0]
    r += cod[1]
    valid = (depth >= 0.1) & (depth <= 12.0) & roimask
    valid = valid.astype(bool)
    z = np.where(valid, depth, np.nan)
    x = np.where(valid, z * (c - CX) / FX, 0)
    y = np.where(valid, z * (r - CY) / FY, 0)
    cloud = np.dstack((x, y, z))
    cloud_roi = cloud[valid]
    return cloud_roi

def cal_centroid(cloud):
    # The following method takes less time than
    # np.mean(cloud, axis=0) or np.sum(cloud, axis=0)/length
    length = cloud.shape[0]
    if not length:
        return np.zeros(3)
    sum_x = np.sum(cloud[:, 0])
    sum_y = np.sum(cloud[:, 1])
    sum_z = np.sum(cloud[:, 2])
    return np.asarray([sum_x, sum_y, sum_z]) / length

def get_predictor(sym, image, arg_params, aux_params):
    data = []
    target_size = config.SCALES[0][0]
    max_size = config.SCALES[0][1]
    im, im_scale = resize(image, target_size, max_size, stride=config.network.IMAGE_STRIDE)
    im_tensor = transform(im, config.network.PIXEL_MEANS)
    im_info = np.array([[im_tensor.shape[2], im_tensor.shape[3], im_scale]], dtype=np.float32)
    data.append({'data': im_tensor, 'im_info': im_info})
    data_names = ['data', 'im_info']
    label_names = []
    data = [[mx.nd.array(data[i][name]) for name in data_names] for i in xrange(len(data))]
    max_data_shape = [[('data', (1, 3, max([v[0] for v in config.SCALES]), max([v[1] for v in config.SCALES])))]]
    provide_data = [[(k, v.shape) for k, v in zip(data_names, data[i])] for i in xrange(len(data))]
    provide_label = [None for i in xrange(len(data))]
    predictor = Predictor(sym, data_names, label_names,
                          context=[mx.gpu(0)], max_data_shapes=max_data_shape,
                          provide_data=provide_data, provide_label=provide_label,
                          arg_params=arg_params, aux_params=aux_params)
    return predictor


def fcis_seg(image, classes, predictor, args):
    num_classes = len(classes) + 1
    data = []
    target_size = config.SCALES[0][0]
    max_size = config.SCALES[0][1]
    im, im_scale = resize(image, target_size, max_size, stride=config.network.IMAGE_STRIDE)
    im_tensor = transform(im, config.network.PIXEL_MEANS)
    start = time.time()
    im_info = np.array([[im_tensor.shape[2], im_tensor.shape[3], im_scale]], dtype=np.float32)
    data.append({'data': im_tensor, 'im_info': im_info})
    data_names = ['data', 'im_info']
    data = [[mx.nd.array(data[i][name]) for name in data_names] for i in xrange(len(data))]
    data_batch = mx.io.DataBatch(data=[data[0]], label=[], pad=0, index=0,
                                 provide_data=[[(k, v.shape) for k, v in zip(data_names, data[0])]],
                                 provide_label=[None])
    scales = [data_batch.data[i][1].asnumpy()[0, 2] for i in xrange(len(data_batch.data))]
    scores, boxes, masks, data_dict = im_detect(predictor, data_batch, data_names, scales, config)
    im_shapes = [data_batch.data[i][0].shape[2:4] for i in xrange(len(data_batch.data))]

    if not config.TEST.USE_MASK_MERGE:
        all_boxes = [[] for _ in xrange(num_classes)]
        all_masks = [[] for _ in xrange(num_classes)]
        nms = py_nms_wrapper(config.TEST.NMS)
        for j in range(1, num_classes):
            indexes = np.where(scores[0][:, j] > 0.7)[0]
            cls_scores = scores[0][indexes, j, np.newaxis]
            cls_masks = masks[0][indexes, 1, :, :]
            try:
                if config.CLASS_AGNOSTIC:
                    cls_boxes = boxes[0][indexes, :]
                else:
                    raise Exception()
            except:
                cls_boxes = boxes[0][indexes, j * 4:(j + 1) * 4]

            cls_dets = np.hstack((cls_boxes, cls_scores))
            keep = nms(cls_dets)
            all_boxes[j] = cls_dets[keep, :]
            all_masks[j] = cls_masks[keep, :]
        dets = [all_boxes[j] for j in range(1, num_classes)]
        masks = [all_masks[j] for j in range(1, num_classes)]
    else:
        masks = masks[0][:, 1:, :, :]
        im_height = np.round(im_shapes[0][0] / scales[0]).astype('int')
        im_width = np.round(im_shapes[0][1] / scales[0]).astype('int')
        # print (im_height, im_width)
        boxes = clip_boxes(boxes[0], (im_height, im_width))
        result_masks, result_dets = gpu_mask_voting(masks, boxes, scores[0], num_classes,
                                                    100, im_width, im_height,
                                                    config.TEST.NMS, config.TEST.MASK_MERGE_THRESH,
                                                    config.BINARY_THRESH, 0)

        dets = [result_dets[j] for j in range(1, num_classes)]
        masks = [result_masks[j][:, 0, :, :] for j in range(1, num_classes)]
    cods, bimsks, names = decode_mask(im, dets, masks, classes, config, args)
    return cods, bimsks, names

def decode_mask(im, detections, masks, class_names, cfg, args, scale=1.0):
    cods = []
    bimsks = []
    names = []
    for j, name in enumerate(class_names):
        if name == '__background__':
            continue
        keep = np.where(detections[j][:, -1] > args.seg_threshold)
        dets = detections[j][keep]
        msks = masks[j][keep]
        for det, msk in zip(dets, msks):
            bbox = det[:4] * scale
            cod = bbox.astype(int)
            if im[cod[1]:cod[3], cod[0]:cod[2], 0].size > 0:
                msk = cv2.resize(msk, im[cod[1]:cod[3] + 1, cod[0]:cod[2] + 1, 0].T.shape)
                bimsk = msk >= cfg.BINARY_THRESH
                bimsk = bimsk.astype(int)
                cods.append(cod)
                bimsks.append(bimsk)
                names.append(name)
    return cods, bimsks, names

def main(args):
    kwargs = {'host': 'localhost',
              'port': 7200}
    pprint.pprint(config)
    sym_instance = eval(config.symbol)()
    sym = sym_instance.get_symbol(config, is_train=False)
    arg_params, aux_params = load_param(args.model_dir, args.trained_epoch, process=True)

    # set up class names
    # num_classes = 81
    classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
               'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
               'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
               'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
               'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
               'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
               'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
               'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
               'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
    # num_classes = 82
    # classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    #            'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
    #            'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
    #            'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    #            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
    #            'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
    #            'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    #            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
    #            'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush', 'floor']
    warmup_img = './warmup.jpg'
    im = cv2.imread(warmup_img, cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    predictor = get_predictor(sym, im, arg_params, aux_params)
    fcis_seg(im, classes, predictor, args)

    server = Server(**kwargs)

    while True:
        try:
            color_imgs = server.get_images()
            depth_imgs = server.get_images()
            color_img = color_imgs[0]
            depth_img = depth_imgs[0]
            cods, bimsks, names = fcis_seg(color_img, classes, predictor, args)#fcis_seg(sym, color_img, classes, arg_params, aux_params, args)
            seg_result = {}
            for cod, bimsk, name in zip(cods, bimsks, names):
                if name not in seg_result:
                    seg_result[name] = []
                depth_roi = depth_img[cod[1]:cod[3] + 1, cod[0]:cod[2] + 1, 0]
                cloud = generate_point_cloud(depth_roi, bimsk, cod)
                # color_roi = color_img[cod[1]:cod[3] + 1, cod[0]:cod[2] + 1, :]
                # show_mask(color_roi, bimsk, name)
                centroid = cal_centroid(cloud)
                seg_result[name].append(centroid)
            server.send_seg_result(seg_result)
        except SocketError as e:
            if e.errno == errno.ECONNRESET:
                print("Connection reset by peer, waiting for reconnnection...")
                server.setup_connect_server()
            else:
                print(e)
                sys.exit("Unexpected socket error occurred")
        except Exception as e:
            print(e)
            sys.exit("Unexpected error occurred")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Segmentation for SLAM')
    parser.add_argument('--model_dir', '-md', type=str, default='./../../model/fcis_coco',
                        help='trained model path')
    parser.add_argument('--trained_epoch', '-te', type=int, default=8, help='how many epochs '
                                                                            'have been trained '
                                                                            'on the trained model')
    parser.add_argument('--seg_threshold', '-sh', type=float, default=0.75,
                        help='threshold value for a successful segmentation')
    args = parser.parse_args()
    main(args)
