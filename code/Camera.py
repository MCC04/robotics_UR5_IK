from math import *
import numpy as np

class Camera(object):
    """
    Utility class for converting between measures in the Field of View
    """

    def __init__(self):
        # sx and sy denote the image size (px)
        # note: not to be confused with the number of pixels per millimeter on the camera sensor.
        self._calib_sx = 800
        self._calib_sy = 800
        # fx and fy denote the focal length (px)
        # (physical focal length (mm) * pixel per mm on the sensor in axis x/y (px/mm))
        # if the pixels are square (which we assume being the case) the two values are equivalent.
        #TEST1
        #self._calib_fx = 485
        #self._calib_fy = 900
        #TEST2
        #self._calib_fx = 475
        #self._calib_fy = 600
        #TEST3
        self._calib_fx = 1200
        self._calib_fy =860
        #TEST4
        #self._calib_fx = 1000
        #self._calib_fy = 200
        # cx and cy denote the pixel coordinates of the center pixel on the camera (px)
        #TEST1
        #self._calib_cx = 400
        #self._calib_cy = 300
        #TEST2
        #self._calib_cx = 400
        #self._calib_cy = 200
        #TEST3
        self._calib_cx = 350
        self._calib_cy = 260
        #TEST4
        #self._calib_cx = 270
        #self._calib_cy =330

        # cx, cy, fx and fy are published on the camera_info topic of the camera at the "K" entry.
        # K denotes the camera matrix and is structured as follows
        # | fx  0.  cx |
        # | 0.  fy  cy |
        # | 0.  0.  1. |

        self._curr_cx = self._curr_cy = None
        self._curr_fx = self._curr_fy = None
        self._curr_sx = self._curr_sy = None

        self.set_image_size(800, 800)

    def set_image_size(self, w, h):
        """
        Sets the image size

        :param w: image's new width
        :param h: image's new height
        """
        if w <= 0 or h <= 0:
            return False
        self._curr_sx = w
        self._curr_sy = h
        scale_x = float(self._curr_sx) / self._calib_sx
        scale_y = float(self._curr_sy) / self._calib_sy
        self._curr_fx = self._calib_fx * scale_x
        self._curr_cx = self._calib_cx * scale_x
        self._curr_fy = self._calib_fy * scale_y
        self._curr_cy = self._calib_cy * scale_y
        return True

    def pixel2metric(self, u, v):
        """
        Converts a pixel coordinate on the image to a metric distance from its center.

        :param u: the x coordinate (column) of the pixel (px)
        :param v: the y coordinate (row) of the pixel (px)
        :return: a pair (xm, ym) denoting the metric distance from the center of the image
            (pure number)
        """

        print "___pixel2metric: ",float(u - self._curr_cx) / self._curr_fx, float(v - self._curr_cy) / self._curr_fy

        return float(u - self._curr_cx) / self._curr_fx, float(v - self._curr_cy) / self._curr_fy

    def metric2pixel(self, xm, ym):
        """
        Converts a metric distance from the image center to a pixel coordinate.

        :param xm: the x distance from the center (pure number)
        :param ym: the y distance from the center (pure number)
        :return: a pair (u, v) denoting a pixel's coordinates (px)
        """
        return xm * self._curr_fx + self._curr_cx, ym * self._curr_fy + self._curr_cy

    @staticmethod
    def metric2angle(xm, ym):
        """
        Converts a metric distance from the image's center to an (azimuth, elevation) pair.

        :param xm: the x distance from the center (pure number)
        :param ym: the y distance from the center (pure number)
        :return: a pair (a, e) denoting the azimuth and the elevation, the center being (0, 0)
            (deg)
        """

        _rho = sqrt(xm ** 2 + ym ** 2 + 1)
        #_rho = 2.2027
        print "____rho:",  _rho
        #a = atan(xm) * 180 / np.pi  # azimuth
        #e = asin(ym / _rho) * 180 / np.pi  # elevation
        a = atan(xm)   # azimuth
        e = asin(ym / _rho)  # elevation
        #e = acos(1 / _rho) * 180 / np.pi  # elevation
        return a, e

    @staticmethod
    def angle2metric(a, e):
        """
        Converts an (azimuth, elevation) pair to a metric distance from the image's center.

        :param a: the azimuth angle from the image's center (deg)
        :param e: the elevation angle from the image's center (deg)
        :return: a pair (xm, ym) denoting the metric distance from the center of the image
            (pure number)
        """
        xm = -tan(a * np.pi / 180)
        ym = -tan(e * np.pi / 180) *sqrt(xm ** 2 + 1)
        return xm, ym

    def pixel2angle(self, u, v):
        """
        Converts a pixel coordinate on the image to an (azimuth, elevation) pair.

        :param u: the x coordinate (column) of the pixel (px)
        :param v: the y coordinate (row) of the pixel (px)
        :return: a pair (a, e) denoting the azimuth and the elevation, the center being (0, 0)
            (deg)
        """
        xm, ym = self.pixel2metric(u, v)
        a, e = Camera.metric2angle(xm, ym)
        return a, e

    def angle2pixel(self, a, e):
        """
        Converts an (azimuth, elevation) pair to a pixel coordinate on the image.

        :param a: the azimuth angle from the image's center (deg)
        :param e: the elevation angle from the image's center (deg)
        :return: a pair (u, v) denoting a pixel's coordinates (px)
        """
        xm, ym = Camera.angle2metric(a, e)
        u, v = self.metric2pixel(xm, ym)
        return u, v

    def pixel2norm(self, u, v):
        """
        Converts a pixel coordinate on the image to a normalized distance from its center.

        :param u: the x coordinate (column) of the pixel (px)
        :param v: the y coordinate (row) of the pixel (px)
        :return: a pair (x, y) denoting the distance from the center of the image
            (pure number in range [-1, 1] x [-1, 1])
        """
        x = 2 * u / self._curr_sx - 1
        y = 2 * v / self._curr_sy - 1
        return x, y

    def norm2pixel(self, x, y):
        """
        Converts a normalized distance from the image's center to the corresponding pixel.

        :param x: the x coordinate of the distance from the center (pure number in range [-1, 1])
        :param y: the y coordinate of the distance from the center (pure number in range [-1, 1])
        :return: a pair (u, v) denoting a pixel's coordinates (px)
        """
        u = self._curr_sx * (x + 1) / 2
        v = self._curr_sy * (y + 1) / 2
        return u, v

    def norm2angle(self, x, y):
        """
        Converts a normalized distance from the image's center to the corresponding
            (azimuth, elevation) pair.

        :param x: the x coordinate of the distance from the center (pure number in range [-1, 1])
        :param y: the y coordinate of the distance from the center (pure number in range [-1, 1])
        :return: a pair (a, e) denoting the azimuth and the elevation, the center being (0, 0)
            (deg)
        """
        u, v = self.norm2pixel(x, y)
        a, e = self.pixel2angle(u, v)
        return a, e

    def norm2metric(self, x, y):
        """
        Converts a normalized distance from the image's center to a metric distance from its center

        :param x: the x coordinate of the distance from the center (pure number in range [-1, 1])
        :param y: the y coordinate of the distance from the center (pure number in range [-1, 1])
        :return: a pair (xm, ym) denoting the metric distance from the center of the image
            (pure number)
        """
        u, v = self.norm2pixel(x, y)
        xm, ym = self.pixel2metric(u, v)
        return xm, ym

    @property
    def height(self):
        """
        Gets the image's height (px)
        """
        return self._curr_sy

    @property
    def width(self):
        """
        Gets the image's width (px)
        """
        return self._curr_sx

    @property
    def cal_height(self):
        """
        Gets the image's height computed during calibration (px)
        """
        return self._calib_sy

    @property
    def cal_width(self):
        """
        Gets the image's width computed during calibration (px)
        """
        return self._calib_sx

    @property
    def fx(self):
        """
        Gets the image's x focal length (px)
        """
        return self._curr_fx

    @property
    def fy(self):
        """
        Gets the image's y focal length (px)
        """
        return self._curr_fy

    @property
    def cx(self):
        """
        Gets the image center's x coordinate (px)
        """
        return self._curr_cx

    @property
    def cy(self):
        """
        Gets the image center's y coordinate (px)
        """
        return self._curr_cy

    @property
    def cal_fx(self):
        """
        Gets the image's x focal length computed during calibration (px)
        """
        return self._calib_fx

    @property
    def cal_fy(self):
        """
        Gets the image's y focal length computed during calibration (px)
        """
        return self._calib_fy

    @property
    def cal_cx(self):
        """
        Gets the image center's x coordinate computed during calibration (px)
        """
        return self._calib_cx

    @property
    def cal_cy(self):
        """
        Gets the image center's y coordinate computed during calibration (px)
        """
        return self._calib_cy
