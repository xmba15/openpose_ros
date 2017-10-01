#!/usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import cv2

def calcMedian(m, nbins, minVal, maxVal, mask):
    """
    calculate the median value of a 2-d mat

    """
    channels = [0]
    histSize = [nbins]
    ranges = [minVal, maxVal]
    hist = cv2.calcHist([m], channels, mask, histSize, ranges)
    
    step = (maxVal - minVal) * 1.0 / nbins
    sumHist = sum(hist)[0]
    median = 0.0
    _sum = 0

    for i in range(nbins):
        _sum += hist[i][0]
        median = minVal + (i * step) + (step / 2.0)
        if (_sum >= (sumHist / 2.0)):
            break

    return median

if __name__=="__main__":
    img = cv2.imread("nature.jpg")
    median = calcMedian(img, 25, 0, 256, None)
    print median



      
