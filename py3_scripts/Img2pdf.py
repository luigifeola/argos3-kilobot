#!/usr/bin/env python
# coding: utf-8

import os
import sys
import img2pdf
from PIL import Image

def print_help():
    print("usage : plot_folder_path")

def main():
    
    number_of_args=len(sys.argv)

    if (number_of_args != 2):
        print_help()
        exit(-1)

    folder=sys.argv[1]
#     folder = "/home/luigi/Documents/scripts/test_scripts/py3_scripts/Plots/2020-02-24"
    if not os.path.isdir(folder):
        print_help()
        exit(-1)
    
    imagelist = []
    for dirName, subdirList, fileList in os.walk(folder):
        fileList.sort()
        for i in fileList:
            if(i.endswith(".png")):

                img = Image.open(dirName+'/'+i)
                img = img.convert('RGB')
                imagelist.append(img)
    
    imagelist[0].save(folder+'/plots.pdf',save_all=True, append_images=imagelist)



if __name__ == '__main__':
    main()

