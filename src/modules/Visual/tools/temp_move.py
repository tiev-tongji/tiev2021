import os
import glob
import shutil


root = '/DataSets2/small_arrow_picked_ApolloScapeLaneBEV/'

img_path = glob.glob('{:s}/**/*.png'.format(os.path.join(root, 'ColorImage_road04')), recursive=True)

for ii, img in enumerate(img_path):
    mask = img.replace('ColorImage/', 'Label/').replace('ColorImage', 'Labels').replace('_bv', '_bin-Bid_GtrainID_RcatId_bv')
    if ii < 5810:
        tar_img = img.replace('ColorImage_road04', 'ColorImage_road04_valid')
        tar_mask = mask.replace('Labels_road04', 'Labels_road04_valid')
        tar_img_dir = os.path.dirname(tar_img)
        tar_mask_dir = os.path.dirname(tar_mask)
        if not os.path.exists(tar_img_dir):
            os.makedirs(tar_img_dir)

        if not os.path.exists(tar_mask_dir):
            os.makedirs(tar_mask_dir)

        shutil.move(img, tar_img_dir)
        shutil.move(mask, tar_mask_dir)

