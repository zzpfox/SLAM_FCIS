from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
import shutil
import os
import glob
import hickle as hkl
import cv2
import re
import argparse
import cPickle as pkl
try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET


def main(args):
    cur_dir = os.path.dirname(os.path.realpath(__file__))
    root_dir = os.path.dirname(os.path.dirname(cur_dir))
    data_dir = os.path.join(root_dir, 'data', 'slxrobot')
    anno_path = os.path.join(data_dir, 'Annotations')
    image_path = os.path.join(data_dir, 'Images')
    mask_path = os.path.join(data_dir, 'Masks')
    mask_dest_path = os.path.join(root_dir, 'data', 'cache', 'slxrobot', 'Masks')
    if not os.path.exists(mask_dest_path):
        os.makedirs(mask_dest_path)

    classes = ['__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
               'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
               'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
               'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
               'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
               'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
               'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
               'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
               'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush', 'floor']
    cls_to_id = dict(zip(classes, range(len(classes))))
    id_to_cls = dict(zip(range(len(classes)), classes))
    image_list = os.listdir(image_path)
    gt_sdsdb = []
    for image in image_list:
        image_idx = image.split('.')[0]
        try:
            image_i = int(image_idx)
        except:
            continue
        anno_file = os.path.join(anno_path, '%s.xml' % image_idx)
        if not os.path.exists(anno_file):
            continue

        tree = ET.ElementTree(file=anno_file)
        bool_masks = []
        boxes = []
        gt_classes = []
        height = int(tree.find('imagesize').find('nrows').text)
        width = int(tree.find('imagesize').find('ncols').text)
        for object in tree.iter('object'):
            seg = object.find('segm')
            deleted = object.find('deleted')
            if int(deleted.text) == 1 or seg is None:
                continue
            cls = object.find('name').text
            if cls in cls_to_id:
                gt_classes.append(cls_to_id[cls])
            else:
                print('Class [%s] is not in the training classes, ignoring it ...' % cls)
                continue
            box = seg.find('box')
            x1 = int(box.find('xmin').text)
            y1 = int(box.find('ymin').text)
            x2 = int(box.find('xmax').text)
            y2 = int(box.find('ymax').text)
            x1 = np.max((0, x1))
            y1 = np.max((0, y1))
            x2 = np.min((width - 1, x2))
            y2 = np.min((height - 1, y2))
            mask = seg.find('mask')
            mask_text = mask.text
            mask_file = os.path.join(mask_path, mask_text)
            mask_im = cv2.imread(mask_file)
            gray_mask = cv2.cvtColor(mask_im, cv2.COLOR_BGR2GRAY)
            bool_mask = np.full(gray_mask.shape, False, dtype=bool)
            mask_row, mask_col = np.where(gray_mask > 1)
            bool_mask[mask_row, mask_col] = True
            bool_masks.append(bool_mask)
            boxes.append([x1, y1, x2, y2])
        gt_classes = np.asarray(gt_classes)
        bool_masks = np.asarray(bool_masks)
        boxes = np.asarray(boxes)
        gt_overlaps = np.zeros((gt_classes.size, len(classes)), dtype=np.float32)
        gt_overlaps[np.arange(gt_overlaps.shape[0]), gt_classes] = 1
        max_overlaps = gt_overlaps.max(axis=1)

        gt_mask_file = os.path.join(mask_dest_path, '%s.hkl' % image_idx)
        gt_mask_flip_file = os.path.join(mask_dest_path, '%s_flip.hkl' % image_idx)
        if not os.path.exists(gt_mask_file):
            print('Saving %s' % gt_mask_file)
            hkl.dump(bool_masks.astype('bool'), gt_mask_file, mode='w', compression='gzip')
        if not os.path.exists(gt_mask_flip_file):
            print('Saving %s' % gt_mask_flip_file)
            hkl.dump(bool_masks[:, :, ::-1].astype('bool'), gt_mask_flip_file, mode='w', compression='gzip')

        sdb = {'boxes': boxes,
               'cache_seg_inst': '%s/%s.hkl' % (os.path.relpath(mask_dest_path, cur_dir), image_idx),
               'flipped': False,
               'gt_classes': gt_classes,
               'gt_overlaps': gt_overlaps,
               'height': height,
               'width': width,
               'image': '%s/%s' % (os.path.relpath(image_path, cur_dir), image),
               'max_classes': gt_overlaps.argmax(axis=1),
               'max_overlaps': max_overlaps,
               }
        gt_sdsdb.append(sdb)
        if args.flip:
            sdb = {'boxes': boxes,
                   'cache_seg_inst': '%s/%s.hkl' % (os.path.relpath(mask_dest_path, cur_dir), image_idx),
                   'flipped': True,
                   'gt_classes': gt_classes,
                   'gt_overlaps': gt_overlaps,
                   'height': height,
                   'width': width,
                   'image': '%s/%s' % (os.path.relpath(image_path, cur_dir), image),
                   'max_classes': gt_overlaps.argmax(axis=1),
                   'max_overlaps': max_overlaps,
                   }
            gt_sdsdb.append(sdb)
    gt_sdsdb_file = os.path.join(mask_dest_path, 'gt_sdsdb.pkl')
    with open(gt_sdsdb_file, 'wb') as f:
        print('Length of gt_sdsdb:', len(gt_sdsdb))
        pkl.dump(gt_sdsdb, f, protocol=pkl.HIGHEST_PROTOCOL)
        # hkl.dump(gt_sdsdb, f, compression='gzip')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert data generated by LabelMe to FCIS format')
    parser.add_argument('--flip', '-f', type=bool, default=True,
                        help='whether to flip the image and mask')
    args = parser.parse_args()
    main(args)




