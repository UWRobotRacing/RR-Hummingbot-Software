from __future__ import print_function
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from PIL import Image

# This script goes through the panoptic file and creates a new json file containing
# only necessary information (brings json file size from 118 MB to 4 MB):
# -> file name
# -> bounding boxes of signs
# New data structure is a list of dicts with each dict corresponding to a different
# file. Each dict has a key for the image id and for a list of bounding boxes.
# bbox = [left_x, top_y, width, height]

def main():

    # read in panoptic file
    with open("training/panoptic/panoptic_2018.json") as panoptic_file:
        panoptic = json.load(panoptic_file)

    data = []
    for file_obj in panoptic["annotations"]:
        # get id and segments from dict
        image_id = file_obj["image_id"]
        segments = file_obj["segments_info"]

        # keep bbox of front traffic signs only (id=51)
        sign_segments = [seg['bbox'] for seg in segments if seg['category_id']==51]

        # convert to dict and add to data
        data_dict = {"image_id": image_id, "signs": sign_segments}
        data.append(data_dict)

    # write data to json file
    with open('label_data.json', 'w') as outfile:
        json.dump(data, outfile)
        
if __name__ == "__main__":
    main()