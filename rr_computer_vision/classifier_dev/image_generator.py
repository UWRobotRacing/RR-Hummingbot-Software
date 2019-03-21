import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt

import os
import json

# constants
MIN_SIGN_SIZE = 20
ASPECT_RATIO_TOLERANCE = 0.25 # aspect ratio must be 1 +/- tolerance
MAX_ROTATION = (0.8, 0.8, 0.05) # max rotation angles in rads (x,y,z) (0.15,0.15,0.15)

DATASET_LOC = "C:/Users/ethie/OneDrive - University of Waterloo/Documents/UWRobotics/StreetImageDataset/mapillary-vistas-dataset_public_v1.1/"

# Notes:
# Zed output is currently set to 720p (1280x720)

def check_aspects(signs):
    # returns signs that have aspect ratio within 1 +/- tolerance value
    if len(signs) == 0:
        return []
    else:
        return ([bbox for bbox in signs if 1 - ASPECT_RATIO_TOLERANCE <= 
                bbox[2]/bbox[3] <= 1 + ASPECT_RATIO_TOLERANCE])

def get_focal(w, h, gamma):
    # calculates optimal focal length for z
    d = np.sqrt(h**2 + h**2)
    focal = d / (2 * np.sin(gamma) if np.sin(gamma) != 0 else 1)

    return focal

def get_perspective_mat(target_size, x_angle, y_angle, z_angle, x, y):
    # reference:
    # https://github.com/eborboihuc/rotate_3d/blob/master/image_transformer.py

    width = target_size
    height = target_size

    focal = get_focal(width, height, z_angle)

    # for translation matrix
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
                    [0, np.cos(x_angle), -np.sin(x_angle), 0],
                    [0, np.sin(x_angle), np.cos(x_angle), 0],
                    [0, 0, 0, 1]])
    
    RY = np.array([ [np.cos(y_angle), 0, -np.sin(y_angle), 0],
                    [0, 1, 0, 0],
                    [np.sin(y_angle), 0, np.cos(y_angle), 0],
                    [0, 0, 0, 1]])
    
    RZ = np.array([ [np.cos(z_angle), -np.sin(z_angle), 0, 0],
                    [np.sin(z_angle), np.cos(z_angle), 0, 0],
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

def apply_transforms(template, target_size, window_size):

    # generate random rotation angles for x,y,z axes (in radians)
    x_angle = np.random.uniform(-MAX_ROTATION[0], MAX_ROTATION[0])
    y_angle = np.random.uniform(-MAX_ROTATION[1], MAX_ROTATION[1])
    z_angle = np.random.uniform(-MAX_ROTATION[2], MAX_ROTATION[2])

    # calculate dx,dy needed to center template
    dx = window_size/2 - target_size/2
    dy = window_size/2 - target_size/2
    
    # calculate perspective matrix for rotations
    mat = get_perspective_mat(window_size, x_angle, y_angle, z_angle, dx, dy)

    # apply transformation matrix
    rotated = cv2.warpPerspective(template.copy(), mat, (window_size, window_size))



    return rotated



def create_data():

    # read in label_data file
    with open('label_data.json') as label_file:
        label_data = json.load(label_file)

    # create haar_training_data folder
    #if not os.path.exists("haar_training_data"):
    #    os.makedirs("haar_training_data/negatives")
    #    os.makedirs("haar_training_data/positives")

    # load in templates
    up_template = cv2.imread("templates/SIGN_UP.jpg")
    h, w, _ = up_template.shape

    M90 = cv2.getRotationMatrix2D((w/2,h/2), 90, 1.0)
    left_template = cv2.warpAffine(up_template, M90, (h, w))

    M270 = cv2.getRotationMatrix2D((w/2,h/2), 270, 1.0)
    right_template = cv2.warpAffine(up_template, M270, (h, w))

    #########################
    image_id = "_m7i-oQEplsprwl-XsPxPQ"
    signs = [[1242, 447, 7, 6], [1186, 458, 10, 7], [1057, 494, 32, 42]]

    image_path = DATASET_LOC + "training/images/{}.jpg".format(image_id)
    base_image = cv2.imread(image_path)

    height, width, _ = base_image.shape
    
    # get signs that have aspect ratio within tolerance
    good_signs = check_aspects(signs)

    if len(good_signs) == 0:
        return False
        # save as negative image
        #neg_path = "haar_training_data/negatives/{}.jpg".format(image_id)
        #cv2.imwrite(neg_path, base_image)

    else:
        # need to pick best sign out of candidates
        # sort by bbox area and pick largest sign
        ratio_sorted = sorted(good_signs, key=lambda sign: sign[2]*sign[3] , reverse=True)
        target_bbox = ratio_sorted[0]

        # size of template (needs to cover old sign)
        target_size = int(max(target_bbox[2], target_bbox[3]) * 1.2)

        # randomly generate -1, 0, or 1 for orientation
        orientation = np.random.randint(-1, 2)

        # choose appropriate template
        # if orientation == -1:
        #     temp = left_template
        # elif orientation == 0:
        #     temp = up_template
        # else:
        #     temp = right_template
        temp = up_template

        template = cv2.resize(temp,(target_size,target_size),interpolation=cv2.INTER_AREA)

        # invert template
        gs_temp = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        gs_temp = cv2.bitwise_not(gs_temp)
        template = cv2.cvtColor(gs_temp, cv2.COLOR_GRAY2BGR)

        # apply random transformations to template
        window_size = int(target_size*1.5)
        transformed_template = apply_transforms(template, target_size, window_size)

        #cv2.imshow('image', transformed_template)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()




    """
    for file_obj in label_data:
        image_id = file_obj["image_id"]
        signs = file_obj["signs"]

        image_path = "training/images/{}.jpg".format(image_id)
        base_image = cv2.imread(image_path)

        height, width, _ = base_image.shape
        
        # get signs that have aspect ratio within tolerance
        good_signs = check_aspects(signs)

        if len(good_signs) == 0:
            # save as negative image
            #neg_path = DATASET_LOC + "haar_training_data/negatives/{}.jpg".format(image_id)
            #cv2.imwrite(neg_path, base_image)

        else:
            # need to pick best sign out of candidates
            # sort by bbox area and pick largest sign
            ratio_sorted = sorted(good_signs, key=lambda sign: sign[2]*sign[3] , reverse=True)
            target_bbox = ratio_sorted[0]

            # size of template after perspective transform (needs to cover old sign)
            target_size = max(target_bbox[2], target_bbox[3]) * 1.1

            # randomly generate -1, 0, or 1 for orientation
            orientation = np.random.randint(-1, 2)

            # choose appropriate template
            if orientation == -1:
                template = left_template
            elif orientation == 0:
                template = up_template
            else:
                template = right_template

            # apply random transformations to template
            transformed_template = apply_transforms(template, target_size)


            cv2.imshow('image', transformed_template)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


            # save as positive image
            #pos_path = DATASET_LOC + "haar_training_data/positives/{}.jpg".format(image_id)
            #cv2.imwrite(pos_path, transformed_img)

"""

    return transformed_template


if __name__ == "__main__":
    temp_trans = create_data()
    print(temp_trans.shape)

    cv2.imshow('image', temp_trans)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
