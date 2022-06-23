#!/usr/bin/env python3

import matplotlib
from PIL import Image
import numpy as np

if __name__ == "__main__":
    im_frame = Image.open('./maze_edited.png')
    np_frame = np.array(im_frame.getdata())
    print(np_frame)