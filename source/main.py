#!/usr/bin/env python

from pathlib import Path

from core.config_constants import G3DConstants
from core.builder import G3DBuilder

if __name__ == '__main__':
    print("Gatherer3D v2 Starts. \n Press Ctrl+C to Exit.")

    #   set calib config
    # main_config = G3DConstants.calib_config2

    # set scan config
    main_config = G3DConstants.scan_config2

    # create input and output fodler if not exist
    Path(main_config.input_folder).mkdir(parents=True, exist_ok=True)
    Path(main_config.output_folder).mkdir(parents=True, exist_ok=True)

    # build and run G3D pipeline
    g3d_builder = G3DBuilder(main_config)
    loop = g3d_builder.build_loop()
    loop.run()
