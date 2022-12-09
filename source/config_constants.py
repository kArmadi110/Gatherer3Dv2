from config_types import *


class Constants:
    recording_config = Config(
        input_mode=InputMode.CAMERA,
        output_mode=field(default_factory=lambda: [OutputMode.VIDEO, OutputMode.STREAM]),
        calib_mode=CalibMode.IMPORT,

        input_folder="./02_out/",
        output_path="./02_out/",
        output_file="YODA",
        input_name="/dev/video0",

        resolution=(1640, 1232),
        input_fps=15,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        pos_certainty_th=0.1
    )

    scann_config1 = Config(
        input_mode=InputMode.CAMERA,
        output_mode=field(default_factory=lambda: [OutputMode.MESH, OutputMode.STREAM]),
        calib_mode=CalibMode.IMPORT,

        input_folder="./02_out/",
        output_path="./02_out/",
        output_file="YODA",
        input_name="/dev/video0",

        resolution=(1640, 1232),
        input_fps=15,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        pos_certainty_th=0.1
    )

    calib_config1 = Config(
        input_mode=InputMode.CAMERA,
        output_mode=field(default_factory=lambda: [OutputMode.CALIBRATION, OutputMode.STREAM]),
        calib_mode=CalibMode.IMPORT,

        input_folder="./02_out/",
        output_path="./02_out/",
        output_file="YODA",
        input_name="/dev/video0",

        resolution=(1640, 1232),
        input_fps=15,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        pos_certainty_th=0.1
    )

    calib_config2 = Config(
        input_mode=InputMode.VIDEO,
        output_mode=field(default_factory=lambda: [OutputMode.CALIBRATION, OutputMode.STREAM]),
        calib_mode=CalibMode.IMPORT,

        input_folder="./02_out/",
        output_path="./02_out/",
        output_file="",
        input_name="calib.mp4",

        resolution=(1640, 1232),
        input_fps=15,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        pos_certainty_th=0.1
    )

    main_config = None
