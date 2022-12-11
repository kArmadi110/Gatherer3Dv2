#!/usr/bin/env python

from pathlib import Path

from core.config_constants import G3DConstants
from core.builder import G3DBuilder

if __name__ == '__main__':
    print("Gatherer3D v2 Starts. \n Press Ctrl+C to Exit.")

    main_config = G3DConstants.recording_config

    Path(main_config.input_folder).mkdir(parents=True, exist_ok=True)
    Path(main_config.output_folder).mkdir(parents=True, exist_ok=True)

    g3d_builder = G3DBuilder(main_config)

    loop = g3d_builder.build_loop()

    loop.run()
    print(f"{loop._counter} {loop._output_streams[0]._core._counter}")
