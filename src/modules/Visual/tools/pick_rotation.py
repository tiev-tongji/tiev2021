from PIL import Image
import numpy as np
import os
import glob
import shutil
import pdb

root = '/mnt/ExtraDisk/ApolloScapeLaneBEV/ColorImage_road04/'
picked_root = '/mnt/ExtraDisk/arrow_picked_ApolloScapeLaneBEV/ColorImage_road04/'
to_pick_IDs = np.arange(19, 31)
to_pick_IDs = to_pick_IDs[:, None, None]
count = 0

# image_path = os.path.join(root, 'ColorImage_road02')
data_path_list = glob.glob('{:s}/**/*.png'.format(root), recursive=True)

for i, image_path in enumerate(data_path_list):
    percent = round(1.0 * i / len(data_path_list) * 100, 2)
    print('当前进度 : %s [%d/%d]' % (str(percent) + '%', i + 1, len(data_path_list)), end='\r')
    # print('complete percent:%10.8s%s'%(str(percent),'%'), end='\r')
    mask_path = image_path.replace('ColorImage/', 'Label/').replace('ColorImage', 'Labels'). \
        replace('_bv', '_bin-Bid_GtrainID_RcatId_bv')

    img = Image.open(image_path).convert('RGB')
    mask = Image.open(mask_path).convert('RGB')
    mask2 = np.array(mask)

    assert mask2.shape[2] == 3
    mask_1 = mask2[:, :, 1]

    mask_1 = mask_1[None, :, :]
    eq = to_pick_IDs == mask_1
    if np.any(eq):
        count += 1
        image_target_path = image_path.replace('ApolloScapeLaneBEV', 'arrow_picked_ApolloScapeLaneBEV')
        img_target_dir = os.path.dirname(image_target_path)
        if not os.path.exists(img_target_dir):
            os.makedirs(img_target_dir)

        mask_target_path = mask_path.replace('ApolloScapeLaneBEV', 'arrow_picked_ApolloScapeLaneBEV')
        mask_target_dir = os.path.dirname(mask_target_path)
        if not os.path.exists(mask_target_dir):
            os.makedirs(mask_target_dir)

        shutil.copyfile(image_path, image_target_path)
        shutil.copyfile(mask_path, mask_target_path)
        # img.save(image_target_path)
        # mask.save(mask_target_path)

print(count)

