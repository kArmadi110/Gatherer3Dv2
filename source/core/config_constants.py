from core.config_types import Config, InputMode, OutputMode, CalibMode


class G3DConstants:
    recording_config = Config(
        is_async_mode=True,
        async_q_limit=5,

        input_mode=InputMode.CAMERA,
        output_mode=[OutputMode.VIDEO],  # , OutputMode.STREAM
        calib_mode=CalibMode.IMPORT,

        input_folder="./bin/",
        output_folder="./bin/",
        output_name="YODA",
        input_name="/dev/video0",

        resolution=(1640, 1232),
        input_fps=30,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        pos_confidence_th=0.1
    )

    scann_config1 = Config(
        is_async_mode=True,
        async_q_limit=5,

        input_mode=InputMode.CAMERA,
        output_mode=[OutputMode.MESH, OutputMode.STREAM],
        calib_mode=CalibMode.IMPORT,

        input_folder="./bin/",
        output_folder="./bin/",
        output_name="YODA",
        input_name="/dev/video0",

        resolution=(1640, 1232),
        input_fps=15,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        pos_confidence_th=0.1
    )

    calib_config1 = Config(
        is_async_mode=True,
        async_q_limit=5,

        input_mode=InputMode.CAMERA,
        output_mode=[OutputMode.CALIBRATION, OutputMode.STREAM],
        calib_mode=CalibMode.IMPORT,

        input_folder="./bin/",
        output_folder="./bin/",
        output_name="YODA",
        input_name="/dev/video0",

        resolution=(1640, 1232),
        input_fps=15,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        pos_confidence_th=0.1
    )

    calib_config2 = Config(
        is_async_mode=True,
        async_q_limit=5,

        input_mode=InputMode.VIDEO,
        output_mode=[OutputMode.CALIBRATION, OutputMode.STREAM],
        calib_mode=CalibMode.IMPORT,

        input_folder="./bin/",
        output_folder="./bin/",
        output_name="",
        input_name="calib.mp4",

        resolution=(1640, 1232),
        input_fps=15,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        pos_confidence_th=0.1
    )

    main_config = None
