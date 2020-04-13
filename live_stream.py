#!/usr/bin/env python
# -*- coding: utf-8 -*-

from libs import ThermalStream, ThermalVisualization

if __name__ == '__main__':
  thermal = ThermalStream()
  thermal.start()

  vis = ThermalVisualization(thermal)
  try:
    vis.show()

  except KeyboardInterrupt:
    thermal.stop()

