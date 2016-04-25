# !/usr/bin/env python


import cv2
import numpy as np

from itertools import tee, izip
import binascii
import math


def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)


def nothing(kargs=[]):
    pass


class QrFinder():
    cap = None
    img = None

    def try_to_decode(self, candidate, source, target):
        epsilon = 0.01 * cv2.arcLength(candidate, True)
        approx = cv2.approxPolyDP(candidate, epsilon, True)

        # makes sure epsilon gives us a square
        while len(approx) > 4:
            epsilon *= 1.01
            approx = cv2.approxPolyDP(candidate, epsilon, True)

        ### draw on screen
        for a, b in pairwise(approx):
            cv2.line(target, tuple(a[0]), tuple(b[0]), (0, 0, 255), 3)
        cv2.line(target, tuple(approx[-1][0]), tuple(approx[0][0]), (0, 0, 255), 3)


        ## detect center of mass, and each corner before decoding
        center = sum(approx) / 4

        topleft = None
        topright = None
        bottomleft = None
        bottomright = None

        for i in approx:
            if i[0][0] < center[0][0] and i[0][1] < center[0][1]:
                topleft = i
            elif i[0][0] > center[0][0] and i[0][1] < center[0][1]:
                topright = i
            elif i[0][0] < center[0][0] and i[0][1] > center[0][1]:
                bottomleft = i
            elif i[0][0] > center[0][0] and i[0][1] > center[0][1]:
                bottomright = i


        self.center = center[0]
        self.size = [topright[0][0] - topleft[0][0], bottomright[0][1] - topright[0][1]]

        targetperspective = np.float32(
            [[[0, 0]], [[100, 0]], [[100, 100]], [[0, 100]]])  ### use points to calculate perspective matrix
        source_perspective = np.float32([topleft, topright, bottomright, bottomleft])

        matrix = cv2.getPerspectiveTransform(source_perspective, targetperspective)
        cv2.warpPerspective(source, matrix, (100, 100),
                            self.corrected)  ### transforms the image to make the token planas

        bits = []
        gridsize = 8
        step = 100 / (gridsize + 3)
        min, max = cv2.minMaxLoc(self.corrected)[:2]
        avg = (min + max) / 2
        offset = 4

        topleft = 1 if self.corrected[1 * step + offset][1 * step + offset] < avg else 0
        topright = 1 if self.corrected[1 * step + offset][(gridsize + 1) * step + offset] < avg else 0
        bottomright = 1 if self.corrected[(gridsize + 1) * step + offset][(gridsize + 1) * step + offset] < avg else 0
        bottomleft = 1 if self.corrected[(gridsize + 1) * step + offset][1 * step + offset] < avg else 0
        cv2.circle(self.corrected, ( (gridsize + 1) * step + offset, 1 * step + offset), 1, (255, 0, 0), 2)
        ### abort if wrong number of markers
        if topleft + topright + bottomright + bottomleft != 3:
            # print "bad"
            return None

        ### detects need of rotation
        angle = 0
        if not topleft:
            angle = 180
        elif not topright:
            angle = 270
        elif not bottomleft:
            angle = 90

        rotation = cv2.getRotationMatrix2D((50, 50), angle, 1.0)
        self.corrected = cv2.warpAffine(self.corrected, rotation, (100, 100))

        ### only gets here if the number of markers is right

        for y in range(2, gridsize + 2):
            for x in range(2, gridsize + 2):
                bits.append(self.corrected[step * y + 2][step * x])
                cv2.circle(self.corrected, (step * x, step * y), 3, (255, 0, 0), 1)

        text = ""
        for pixel in bits:
            text += "1" if pixel < avg else "0"
        data = [text[i:i + 8] for i in range(0, len(text), 8)]

        result = ""
        cv2.namedWindow('corrected')
        cv2.imshow('corrected', self.corrected)
        data, checksum = data[:-1], data[-1]

        for number in data:
            try:
                n = int('0b' + number, 2)
                result += binascii.unhexlify('%x' % n)
            except:
                pass

        bitstring = '0b'+''.join(data)
        while bitstring.endswith("00000000"):
            bitstring = bitstring[:-8]
        checksumcalculated = bin(sum(map(ord, bitstring)) % 255)
        if checksum == checksumcalculated[2:]:
            print result#, checksum, checksumcalculated

    def __init__(self):
        self.cap = None
        self.corrected = np.zeros((100, 100), np.uint8)  # image with corrected perspective

        cv2.namedWindow('edge')
        cv2.createTrackbar('thrs1', 'edge', 2000, 5000, nothing)
        cv2.createTrackbar('thrs2', 'edge', 4000, 5000, nothing)

        self.center= [0,0]
        self.size = [0,0]


    def find_code(self,img ):
        h, w = img.shape[:2]

        #gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        gray=img
        thrs1 = cv2.getTrackbarPos('thrs1', 'edge')
        thrs2 = cv2.getTrackbarPos('thrs2', 'edge')
        edge = cv2.Canny(gray, thrs1, thrs2, apertureSize=5)
        vis = img.copy()
        vis /= 2
        #vis[edge != 0] = (0, 255, 0)
        #cv2.imshow('edge', vis)

        vis2 = np.zeros((h, w), np.uint8)
        vis2[edge != 0] = 255

        _, contours0, hierarchy = cv2.findContours(vis2.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cv2.approxPolyDP(cnt, 3, True) for cnt in contours0]

        selected = []
        # **[Next, Previous, First_Child, Parent]**
        if hierarchy != None:
            for c, h in zip(contours, hierarchy[0]):
                if h[0] == -1 and h[1] == -1:
                    kid = h[2]
                    if kid != -1:
                        kidh = hierarchy[0][kid]
                        if kidh[0] == -1 and kidh[1] == -1:  ### only checking for nested circles, without brothers
                            selected.append(c)

        cv2.drawContours(vis, selected, -1, (255, 0, 0), 2, cv2.LINE_AA)
        for candidate in selected:
            try:
                self.try_to_decode(candidate, gray, vis)
            except Exception, e:
                print e

        cv2.imshow('contours', vis)

        ch = cv2.waitKey(5) & 0xFF
        if ch == 27:
            exit()

if __name__ == '__main__':
    finder = QrFinder()
