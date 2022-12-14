#!/usr/bin/env python

import sys
from typing import List

import core.config_types as g3d_cfg
from core.g3d_async_output import G3DAsyncOutput
from core.g3d_input import G3DInput
from core.process_loop import ProcessLoop

import inputs as g3d_in
import outputs as g3d_out


class G3DBuilder():
    def __init__(self, cfg: g3d_cfg.Config):
        self._cfg = cfg

    def build_loop(self) -> ProcessLoop:
        return ProcessLoop(self.build_input(), self.build_outputs())

    def build_outputs(self) -> List:
        result = []

        if g3d_cfg.OutputMode.VIDEO in self._cfg.output_mode:
            result.append(g3d_out.CV2Video(self._cfg))
            # result.append(g3d_out.FFMPEGVideo(self._cfg))

        if g3d_cfg.OutputMode.IMAGES in self._cfg.output_mode:
            result.append(g3d_out.Images(self._cfg))

        if g3d_cfg.OutputMode.STREAM in self._cfg.output_mode:
            # result.append(g3d_out.CV2Gstream(self._cfg))
            result.append(g3d_out.FFMPEGStream(self._cfg))

        if g3d_cfg.OutputMode.CALIB_BOARD in self._cfg.output_mode:
            temp = g3d_out.CameraCalibration(self._cfg)
            temp.generate_charuco_plane()

            print("Calibration Data exported!")
            sys.exit(0)

        if g3d_cfg.OutputMode.CALIBRATION in self._cfg.output_mode:
            if g3d_cfg.CalibMode.CAMERA == self._cfg.calib_mode:
                result.append(g3d_out.CameraCalibration(self._cfg))
            elif g3d_cfg.CalibMode.LASER == self._cfg.calib_mode:
                result.append(g3d_out.LaserCalibration(self._cfg))
            elif g3d_cfg.CalibMode.EXPOSURE == self._cfg.calib_mode:
                result.append(g3d_out.ExposureCalibration(self._cfg))

        if g3d_cfg.OutputMode.MESH in self._cfg.output_mode:
            result.append(g3d_out.Mesh(self._cfg))

        if self._cfg.is_async_mode:
            result_wrapper = []
            for out in result:
                result_wrapper.append(G3DAsyncOutput(self._cfg, out))
            return result_wrapper

        return result

    def build_input(self) -> G3DInput:
        result = None

        if self._cfg.input_mode == g3d_cfg.InputMode.CAMERA:
            result = g3d_in.PC2Camera(self._cfg)
            # result = g3d_in.CV2Camera(self._cfg)
        elif self._cfg.input_mode == g3d_cfg.InputMode.IMAGES:
            result = g3d_in.Images(self._cfg)
        elif self._cfg.input_mode == g3d_cfg.InputMode.VIDEO:
            result = g3d_in.Video(self._cfg)

        return result
