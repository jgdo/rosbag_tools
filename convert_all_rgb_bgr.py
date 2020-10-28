import os
import fnmatch
import cv2
from tqdm import tqdm

recordings_parent_dir = '/home/mprediger/recordings_baxter/extracted_images'

for root, dirs, files in os.walk(recordings_parent_dir):
    print('Flipping rgb in {}'.format(root))
    for name in tqdm(files):
        filepath = os.path.join(root, name)
        if name.startswith('frame') and name.endswith('.jpg'):
            img = cv2.imread(filepath)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imwrite(filepath, img)
        # print(name)
        # print(os.path.join(root, name))
