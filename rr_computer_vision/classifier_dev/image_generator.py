import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt

import os
import json
import shutil

# constants
MIN_SIGN_SIZE = 40
ASPECT_RATIO_TOLERANCE = 0.1 # aspect ratio must be 1 +/- tolerance
MAX_ROTATION = (0.8, 0.8, 0.1) # max rotation angles in rads (x,y,z)
BLUR_KERNEL = (1, 1)
GAUSSIAN_NOISE = (0, 0.1) # mean and variance for gaussian noise
#ALPHA = [1.0, 3.0] # Range for random contrast adjustment
BETA = [0, 50] # Range for random brightness adjustment

DATASET_LOC = "C:/Users/ethie/OneDrive - University of Waterloo/Documents/UWRobotics/StreetImageDataset/mapillary-vistas-dataset_public_v1.1/"
NEW_DATASET_LOC = "C:/Users/ethie/OneDrive - University of Waterloo/Documents/UWRobotics/StreetImageDataset/synthetic_data/"

# Notes:
# Zed output is currently set to 720p (1280x720)
# bbox = [left_x, top_y, width, height]

def rescale(image, signs):
    # Rescales base image to 720p and adjusts sign bounding boxes to match

    if image.shape[0] == 720:
        return image, signs
    else:
        h, w, _ = image.shape

        scale_factor = 720 / h
        scaled_img = cv2.resize(image, None, fx=scale_factor, fy=scale_factor)

        for i in range(len(signs)):
            signs[i] = [int(coord * scale_factor) for coord in signs[i]]

        return scaled_img, signs

def get_good_signs(signs, base_shape):
    # Returns signs that have aspect ratio within 1 +/- tolerance value and are 
    # far enough from the edge of the base image
    if len(signs) == 0:
        return []
    else:
        non_zero = [bbox for bbox in signs if bbox[2] != 0 and bbox[3] != 0]
        within_tol = ([bbox for bbox in non_zero if 1 - ASPECT_RATIO_TOLERANCE <= 
                    bbox[2]/bbox[3] <= 1 + ASPECT_RATIO_TOLERANCE])
        target_sizes = [int(max(bbox[2], bbox[3]) * 2.0) for bbox in within_tol]
        good_signs = []
        for i in range(len(target_sizes)):
            bbox = within_tol[i]
            size = target_sizes[i]
            if bbox[0] + bbox[2] - size > 0 and bbox[0] + size < base_shape[1] and bbox[1] + size < base_shape[0] and bbox[1] + bbox[3] - size > 0 and size > MIN_SIGN_SIZE:
                good_signs.append(within_tol[i])
        return good_signs
        

def get_focal(w, h, gamma):
    # calculates optimal focal length for z
    d = np.sqrt(h**2 + h**2)
    focal = d / (2 * np.sin(gamma) if np.sin(gamma) != 0 else 1)

    return focal

def get_perspective_mat(target_size, rotations, x, y):
    # reference:
    # https://github.com/eborboihuc/rotate_3d/blob/master/image_transformer.py

    width = target_size
    height = target_size

    focal = get_focal(width, height, rotations[2])

    # For translation matrix
    dx = x
    dy = y
    dz = focal

    # Projection 2D -> 3D matrix
    A1 = np.array([ [1, 0, -width/2],
                    [0, 1, -height/2],
                    [0, 0, 1],
                    [0, 0, 1]])
    
    # Rotation matrices around the X, Y, and Z axis
    RX = np.array([ [1, 0, 0, 0],
                    [0, np.cos(rotations[0]), -np.sin(rotations[0]), 0],
                    [0, np.sin(rotations[0]), np.cos(rotations[0]), 0],
                    [0, 0, 0, 1]])
    
    RY = np.array([ [np.cos(rotations[1]), 0, -np.sin(rotations[1]), 0],
                    [0, 1, 0, 0],
                    [np.sin(rotations[1]), 0, np.cos(rotations[1]), 0],
                    [0, 0, 0, 1]])
    
    RZ = np.array([ [np.cos(rotations[2]), -np.sin(rotations[2]), 0, 0],
                    [np.sin(rotations[2]), np.cos(rotations[2]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Composed rotation matrix with (RX, RY, RZ)
    R = np.dot(np.dot(RX, RY), RZ)

    # Translation matrix
    T = np.array([  [1, 0, 0, dx],
                    [0, 1, 0, dy],
                    [0, 0, 1, dz],
                    [0, 0, 0, 1]])

    # Projection 3D -> 2D matrix
    A2 = np.array([ [focal, 0, width/2, 0],
                    [0, focal, height/2, 0],
                    [0, 0, 1, 0]])

    # Final transformation matrix
    return np.dot(A2, np.dot(T, np.dot(R, A1)))

def apply_transforms(template, target_size, window_size, rotations):

    # Order of transforms:
    # 1. Hue variations
    # 2. Lighting variations
    # 3. Rotations
    # 4. Gaussian blur
    # 5. Gaussian noise

    # convert template to hsv
    #hsv_template = cv2.cvtColor(template, cv2.BGR2HSV)

    # generate random rotation angles for x,y,z axes (in radians)
    # x_angle = np.random.uniform(-MAX_ROTATION[0], MAX_ROTATION[0])
    # y_angle = np.random.uniform(-MAX_ROTATION[1], MAX_ROTATION[1])
    # z_angle = np.random.uniform(-MAX_ROTATION[2], MAX_ROTATION[2])

    # calculate dx,dy needed to center template
    dx = (window_size - target_size)/2
    dy = (window_size - target_size)/2
    
    # calculate perspective matrix for rotations
    mat = get_perspective_mat(window_size, rotations, dx, dy)

    # apply transformation matrix
    rotated = cv2.warpPerspective(template.copy(), mat, (window_size, window_size), borderValue=255)

    # get binary mask for rotated image
    inverted = cv2.bitwise_not(rotated)
    ret, binary_template = cv2.threshold(inverted, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    _, contours, _ = cv2.findContours(binary_template, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    mask = np.zeros(binary_template.shape, np.uint8)
    cv2.drawContours(mask, [cnt], 0, 255, -1)

    # add gaussian noise
    shp = rotated.shape
    gaussian = np.random.normal(GAUSSIAN_NOISE[0], GAUSSIAN_NOISE[1]**0.5, shp)
    gaussian = gaussian.reshape(shp[0], shp[1])
    noised = rotated + gaussian

    return noised, mask


if __name__ == "__main__":
    # read in label_data file
    #with open('rr_computer_vision/classifier_dev/label_data.json') as label_file:
    with open('label_data.json') as label_file:
        label_data = json.load(label_file)

    # create haar_training_data folder
    if os.path.exists(NEW_DATASET_LOC + "haar_training_data"):
        shutil.rmtree(NEW_DATASET_LOC + "haar_training_data")
    os.makedirs(NEW_DATASET_LOC + "haar_training_data/negatives/images")
    os.makedirs(NEW_DATASET_LOC + "haar_training_data/positives/images")
    pos_file = open(NEW_DATASET_LOC + "haar_training_data/positives/info.dat", "w")
    neg_file = open(NEW_DATASET_LOC + "haar_training_data/negatives/bg.txt", "w")
    
    # load in templates
    #up_template = cv2.imread("rr_computer_vision/classifier_dev/templates/SIGN_UP.jpg", 0)
    up_template = cv2.imread("templates/SIGN_UP.jpg", 0)
    h, w = up_template.shape

    M90 = cv2.getRotationMatrix2D((w/2,h/2), 90, 1.0)
    left_template = cv2.warpAffine(up_template, M90, (h, w))

    M270 = cv2.getRotationMatrix2D((w/2,h/2), 270, 1.0)
    right_template = cv2.warpAffine(up_template, M270, (h, w))

    counter = 1
    for file_obj in label_data:
        print("{}/{}".format(counter, len(label_data)))
        counter+=1
        image_id = file_obj["image_id"]
        image_path = DATASET_LOC + "training/images/{}.jpg".format(image_id)

        prescaled_signs = file_obj["signs"]
        prescaled_image = cv2.imread(image_path)

        base_image, signs = rescale(prescaled_image, prescaled_signs)

        height, width, _ = base_image.shape
        
        # get signs that have aspect ratio within tolerance and are near edge of pic
        good_signs = get_good_signs(signs, (height, width))

        if len(good_signs) == 0:
            # save as negative image
            neg_path = NEW_DATASET_LOC + "haar_training_data/negatives/images/{}.jpg".format(image_id)
            cv2.imwrite(neg_path, base_image)
            neg_file.write("/images/{}.jpg\n".format(image_id))

        else:
            # need to pick best sign out of candidates
            # sort by bbox area and pick largest sign
            ratio_sorted = sorted(good_signs, key=lambda sign: sign[2]*sign[3] , reverse=True)
            target_bbox = ratio_sorted[0]

            # Size of template (needs to cover old sign)
            target_size = int(max(target_bbox[2], target_bbox[3]) * 1.25)

            # Randomly generate -1, 0, or 1 for orientation
            orientation = np.random.randint(-1, 2)

            # Choose appropriate template
            if orientation == -1:
                temp = left_template
            elif orientation == 0:
                temp = up_template
            else:
                temp = right_template

            template = cv2.resize(temp, (target_size,target_size), interpolation=cv2.INTER_AREA)
            
            # Randomly change brightness of template (sample using beta dist)
            array_beta = np.full(template.shape, np.uint8(np.interp(np.random.beta(2, 4, 1), [0, 1], BETA)))
            cv2.add(template, array_beta, template)

            # Calculate rotation angles according to sign location in image
            # bbox = [left_x, top_y, width, height]
            rotations = [] #vertical, horizontal, diagonal
            rotations.append(np.interp(target_bbox[0]+target_bbox[2]/2, [0, base_image.shape[1]], [MAX_ROTATION[0], -MAX_ROTATION[0]])) # x angle
            rotations.append(np.interp(target_bbox[1]+target_bbox[3]/2, [0, base_image.shape[0]], [MAX_ROTATION[1], -MAX_ROTATION[1]])) # y angle
            #rotations.append(np.random.uniform(-MAX_ROTATION[1], MAX_ROTATION[1]))
            rotations.append(np.random.uniform(-MAX_ROTATION[2], MAX_ROTATION[2])) # z angle


            # Apply transformations to template
            window_size = int(target_size*1.5)
            transformed_template, mask = apply_transforms(template, target_size, window_size, rotations)

            # Overlay transformed template over base image
            template_loc = (int(target_bbox[0] + target_bbox[2]/2 - transformed_template.shape[1]/2), int(target_bbox[1] + target_bbox[3]/2 - transformed_template.shape[0]/2))
            rgb_mask = np.dstack((cv2.bitwise_not(mask),)*3)
            
            masked_template = cv2.bitwise_and(transformed_template, transformed_template, mask=mask)
            masked_template = np.dstack((masked_template,)*3)
            
            y1 = template_loc[1]
            y2 = template_loc[1]+transformed_template.shape[0]
            x1 = template_loc[0]
            x2 = template_loc[0]+transformed_template.shape[1]

            image_final = base_image.copy()
            image_final[y1:y2, x1:x2] = cv2.bitwise_and(image_final[y1:y2, x1:x2], rgb_mask)
            image_final[y1:y2, x1:x2] = image_final[y1:y2, x1:x2] + masked_template

            # Convert to 720p


            # Apply blur to sign region
            blurred = cv2.GaussianBlur(image_final[y1:y2, x1:x2], BLUR_KERNEL, 0)
            image_final[y1:y2, x1:x2] = blurred
            
            # save as positive image
            pos_path = NEW_DATASET_LOC + "haar_training_data/positives/images/{}.jpg".format(image_id)
            cv2.imwrite(pos_path, image_final)
            pos_file.write("/images/{}.jpg 1 {} {} {} {}\n".format(image_id, target_bbox[0], target_bbox[1], target_bbox[2], target_bbox[3]))
            
    # cv2.imshow('image',img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
