#-*- encoding: UTF-8 -*-
#导入import模块
from pprint import pprint
import sys
reload(sys)
sys.setdefaultencoding("utf-8")


import numpy as np
import matplotlib.pyplot as plt
import os

def main(arg):
    x_raw = []
    y_raw = []
    t_raw = []

    x_final = []
    y_final = []
    t_final = []

    _x = []
    i = 0
    file_name_raw = 'reeds_shepp_' + arg + '.txt'
    with open(file_name_raw, 'r') as f:
        for l in f:
            x_raw.append(l[2:7])
            _y = l.find('y:')
            y_raw.append(l[_y + 2: _y+6])
            i += 1
            _x.append(i)
        pprint(y_raw)

    plt.figure(12)
    plt.subplot(121)
    plt.plot(x_raw, y_raw, 'g.')

    plt.show()


if __name__ == "__main__":
    main(str(sys.argv[1]))
