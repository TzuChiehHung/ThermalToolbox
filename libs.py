#!/usr/bin/env python
# -*- coding: utf-8 -*-

from uvctypes import *
import time
import cv2
import matplotlib.pyplot as plt
import numpy as np
import platform
import time

class ThermalStream(object):

  def __init__(self):
    self.data = np.zeros((120, 160), dtype=np.uint16)
    self.ptr_py_frame_callback = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(self.py_frame_callback)

    self.ctx = POINTER(uvc_context)()
    self.dev = POINTER(uvc_device)()
    self.devh = POINTER(uvc_device_handle)()
    self.ctrl = uvc_stream_ctrl()

    self.is_active = False

    res = libuvc.uvc_init(byref(self.ctx), 0)
    if res < 0:
      print("uvc_init error")
      exit(1)

  def py_frame_callback(self, frame, userptr, copy=False):
    array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
    if copy:
      # copy
      data = np.fromiter(
        frame.contents.data,
        dtype=np.dtype(np.uint8),
        count=frame.contents.data_bytes)
    else:
      # no copy
      data = np.frombuffer(
        array_pointer.contents,
        dtype=np.dtype(np.uint16))

    self.data = data.reshape(frame.contents.height, frame.contents.width)

    if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
      return

  def ktof(self, val):
    return (1.8 * self.ktoc(val) + 32.0)

  def ktoc(self, val):
    return (val - 27315) / 100.0

  def start(self, disp=False):

    res = libuvc.uvc_find_device(self.ctx, byref(self.dev), PT_USB_VID, PT_USB_PID, 0)
    if res < 0:
      print("uvc_find_device error")
      exit(1)

    res = libuvc.uvc_open(self.dev, byref(self.devh))
    if res < 0:
      print("uvc_open error")
      exit(1)

    print("device opened!")

    if disp:
      print_device_info(self.devh)
      print_device_formats(self.devh)

    frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_Y16)
    if len(frame_formats) == 0:
      print("device does not support Y16")
      exit(1)

    libuvc.uvc_get_stream_ctrl_format_size(
      self.devh, byref(self.ctrl),
      UVC_FRAME_FORMAT_Y16,
      frame_formats[0].wWidth,
      frame_formats[0].wHeight,
      int(1e7 / frame_formats[0].dwDefaultFrameInterval))

    res = libuvc.uvc_start_streaming(
      self.devh,
      byref(self.ctrl),
      self.ptr_py_frame_callback,
      None,
      0)
    if res < 0:
      print("uvc_start_streaming failed: {0}".format(res))
      exit(1)

    self.is_active = True

  def stop(self):
    cv2.destroyAllWindows()
    libuvc.uvc_stop_streaming(self.devh)
    libuvc.uvc_unref_device(self.dev)
    libuvc.uvc_exit(self.ctx)
    self.is_active = False


class ThermalVisualization(object):

  def __init__(self, thermal):
    self.thermal = thermal

  def raw_to_8bit(self, data):
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(data, 8, data)
    return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

  def display_temperature(self, img, val_k, loc, unit='C', color=(255, 255, 255)):
    if unit == 'C':
      val = self.thermal.ktoc(val_k)
      txt = 'degC'
    elif unit == 'F':
      val = self.thermal.ktof(val_k)
      txt = 'degF'
    else:
      val = val_k
      txt = 'K'

    cv2.putText(img,'{0:.1f}'.format(val) + txt, loc, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
    x, y = loc
    cv2.line(img, (x - 2, y), (x + 2, y), color, 1)
    cv2.line(img, (x, y - 2), (x, y + 2), color, 1)

  def show(self, img_size=(640,480), unit='C'):
    while self.thermal.is_active:
      data = cv2.resize(self.thermal.data[:,:], img_size)
      minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)
      img = self.raw_to_8bit(data)
      # img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
      self.display_temperature(img, minVal, minLoc, unit=unit, color=(255, 0, 0))
      self.display_temperature(img, maxVal, maxLoc, unit=unit, color=(0, 0, 255))
      cv2.imshow('Lepton Radiometry', img)

      key = cv2.waitKey(1) & 0xFF
      if key == ord('q'):
        break
      elif key == ord('s'):
        timestr = time.strftime('%Y%m%d_%H%M%S') + '.png'
        cv2.imwrite(timestr, img)
        print('save image as {}'.format(timestr))

    self.thermal.stop()