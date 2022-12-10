#!/usr/bin/env python

from core.config_constants import *
from source.core.builder import G3DBuilder
from pathlib import Path

if __name__ == '__main__':
    print("Gatherer3D v2 Starts. \n Press Ctrl+C to Exit.")

    main_config = G3DConstants.scann_config1

    Path(main_config.input_folder).mkdir(parents=True, exist_ok=True)
    Path(main_config.output_folder).mkdir(parents=True, exist_ok=True)

    g3d_builder = G3DBuilder(main_config)

    loop = g3d_builder.build_loop()

    loop.run()
