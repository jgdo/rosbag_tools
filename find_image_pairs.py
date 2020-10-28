import cv2
import re
import sys
import numpy as np
import PySimpleGUI as sg
import glob
import hickle as hkl
import os

#recordings_parent_dir = '/home/mprediger/recordings_baxter'

#dirs = [name for name in os.listdir(recordings_parent_dir) if os.path.isdir(os.path.join(recordings_parent_dir, name))]

def find_closest_index(value, sorted_list, valfunc):
    assert sorted_list

    start, end = 0, len(sorted_list)

    if value <= valfunc(sorted_list[start]):
        return start
    elif value >= valfunc(sorted_list[end-1]):
        return end-1

    while start + 1 < end:
        mid = (start + end) // 2
        if value < valfunc(sorted_list[mid]):
            end = mid
        else:
            start = mid

    return start

def find_all_closest_indices(list_a, list_b, valfunc):
    best_a = [None] * len(list_a)

    for i_a in range(len(list_a)):
        i_b = find_closest_index(valfunc(list_a[i_a]), list_b, valfunc)
        best_a[i_a] = i_b

    return best_a

def find_best_matches(list_a, list_b, valfunc = lambda x: x):
    best_a = find_all_closest_indices(list_a, list_b, valfunc)
    best_b = find_all_closest_indices(list_b, list_a, valfunc)

    # create pairs of best matches (index in a, index in b)
    matches_a = list(zip(range(len(list_a)), best_a))
    matches_b = list(zip(best_b, range(len(list_b))))

    matches = matches_a + matches_b
    matches = sorted(matches, key=lambda m: abs(valfunc(list_b[m[1]]) - valfunc(list_a[m[0]])))

    done_a = [False] * len(list_a)
    done_b = [False] * len(list_b)

    valid_indices = []
    valid_pairs = []

    for m in matches:
        if not done_a[m[0]] and not done_b[m[1]]:
            valid_indices.append(m)
            valid_pairs.append((list_a[m[0]], list_b[m[1]]))
            done_a[m[0]] = True
            done_b[m[1]] = True

    return valid_pairs


recording_base_folder = "/home/mprediger/recordings_baxter/extracted_images/2019-11-20_try10_Mark_cabinRoofOutOfSequence"

cam_1_image_names = sorted(glob.glob(os.path.join(recording_base_folder, "cam_1/frame*.jpg")))
cam_3_image_names = sorted(glob.glob(os.path.join(recording_base_folder, "cam_3/frame*.jpg")))
assert cam_1_image_names and cam_3_image_names

# extract index from filename
pattern = re.compile(r"_(?P<index>[0-9]+)_(?P<time>[0-9]+)")

cam_1_times = [int(pattern.search(x).group('time')) for x in cam_1_image_names]
cam_3_times = [int(pattern.search(x).group('time')) for x in cam_3_image_names]

cam_1_stamped = list(zip(cam_1_image_names, cam_1_times))
cam_3_stamped = list(zip(cam_3_image_names, cam_3_times))

image_pairs = find_best_matches(cam_1_stamped, cam_3_stamped, lambda e: e[1])

image_pairs = sorted(image_pairs, key=lambda e: e[0][1])

num_images = len(image_pairs)

sg.change_look_and_feel('SystemDefaultForReal')

# define the window layout
layout = [
          [sg.Image(filename='', key='image1', size=(50, 1)), sg.Image(filename='', key='image3', size=(50, 1))],
          [sg.Button('<', size=(10, 1), font='Helvetica 14'),
           sg.Checkbox('Play', font='Any 14', default=False, enable_events=True, key='play'),
           sg.Button('>', size=(10, 1), font='Any 14'),
           sg.Button('Exit', size=(10, 1), font='Helvetica 14'),
           sg.Text('none', key='action_text', font='Any 14'),
           sg.Checkbox('Act current', font='Any 14', default=False, enable_events=True, key='annotate_current'),
           sg.Checkbox('Deact current', font='Any 14', default=False, enable_events=True, key='de_annotate_current'),
           sg.Button('Save', size=(10, 1), font='Helvetica 14')],
          [sg.Slider(range=(0, num_images-1),
                     key='_FRAME_SLIDER_',
                     default_value=0,
                     size=(169, 15),
                     orientation='horizontal',
                     font=('Helvetica', 12))],
          [sg.Image(filename='', key='action_image', size=(100, 1))]
          ]

# create the window and show it without the plot
window = sg.Window('Activity annotation {}'.format(recording_base_folder),
                   layout, location=(1800, 400), return_keyboard_events=True)

image_index = -1
prev_image_index = image_index
image_cache = dict()

is_action = np.zeros((num_images), dtype=np.uint8)

def update_annotation_signal():
    window.Element("action_text").Update('ACT' if is_action[image_index] else 'none',
                                         text_color='red' if is_action[image_index] else 'green')

    is_action_image = 255 - is_action[np.newaxis].astype(int) * 255
    is_action_image = np.repeat(is_action_image, 10, 0)
    is_action_image = np.repeat(is_action_image[:, :, np.newaxis], 3, axis=2)

    # now is_action_image is 255 for every False, and 0 for every True

    is_action_image[:, :, 2] = 255 # turn True to red [0, 0, 255] and False to white [255, 255, 255]

    is_action_image[5:10, image_index] = [0, 150, 0]

    window['action_image'].update(data=cv2.imencode('.png', is_action_image)[1].tobytes())

while True:
    event, values = window.read(timeout=5)
    if event == 'Exit' or event is None:
        sys.exit()

    elif event == '<' or 'Left' in event:
        if image_index > 0:
            image_index -= 1

    elif event == '>' or 'Right' in event:
        if image_index < num_images-1:
            image_index += 1

    elif event == 'de_annotate_current':
        window.Element("annotate_current").Update(False)
        values['annotate_current'] = False

    elif event == 'annotate_current':
        window.Element("de_annotate_current").Update(False)
        values['de_annotate_current'] = False

    elif event == 'Save':
        activity_pairs = [(x[0][0], x[1][0], is_action[i]) for i, x in enumerate(image_pairs)]
        save_path= os.path.join(recording_base_folder, "activity_pairs.hkl")

        if os.path.isfile(save_path):
            if sg.PopupYesNo('PopupYesNo') == 'No':
                continue

        hkl.dump(activity_pairs, save_path)

    else:
        image_index = int(values['_FRAME_SLIDER_'])

    if values['annotate_current']:
        if not is_action[image_index]:
            is_action[image_index] = True
            update_annotation_signal()

    if values['de_annotate_current']:
        if is_action[image_index]:
            is_action[image_index] = False
            update_annotation_signal()

    if values['play'] and image_index < num_images - 1:
        image_index += 1


    if image_index != prev_image_index:
        prev_image_index = image_index
        window.Element("_FRAME_SLIDER_").Update(image_index)

        if image_index in image_cache:
            imgbytes1, imgbytes3 = image_cache[image_index]
        else:
            # this is faster, shorter and needs less includes
            imgbytes1 = cv2.imencode('.png', cv2.imread(image_pairs[image_index][0][0]))[1].tobytes()
            imgbytes3 = cv2.imencode('.png', cv2.imread(image_pairs[image_index][1][0]))[1].tobytes()

            image_cache[image_index] = (imgbytes1, imgbytes3)

            # print('Cache size ', len(image_cache))

        window['image1'].update(data=imgbytes1)
        window['image3'].update(data=imgbytes3)

        update_annotation_signal()
