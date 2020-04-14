#!/usr/bin/env python
# -*- coding: utf-8 -*-

from libs import ThermalStream

if __name__ == '__main__':
  thermal = ThermalStream()
  thermal.start()
  try:
    thermal.show()

  except KeyboardInterrupt:
    thermal.stop()

