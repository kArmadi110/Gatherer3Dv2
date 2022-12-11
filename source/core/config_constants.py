from core.config_types import Config, InputMode, OutputMode, CalibMode, SegmentationMode


class G3DConstants:
    recording_config = Config(
        is_async_mode=True,
        async_q_limit=5,

        input_mode=InputMode.CAMERA,
        output_mode=[OutputMode.VIDEO, OutputMode.STREAM],  #
        calib_mode=CalibMode.IMPORT,

        input_folder="./bin/calib7/",
        output_folder="./bin/calib7/",
        output_name="calib7",
        input_name="/dev/video0",

        resolution=(1640, 1232),
        input_fps=30,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        color_mode=False,
        segmentation_mode=SegmentationMode.SUBPIXEL,
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
        color_mode=False,
        segmentation_mode=SegmentationMode.SUBPIXEL,
        pos_confidence_th=0.1
    )

    calib_config1 = Config(
        is_async_mode=False,
        async_q_limit=5,

        input_mode=InputMode.VIDEO,
        output_mode=[OutputMode.CALIBRATION],
        calib_mode=CalibMode.LASER,

        input_folder="./bin/calib1/",
        output_folder="./bin/calib1/intensity/",
        # output_folder="./bin/calib1/subpixel",
        # output_folder="./bin/calib1/middle",
        output_name="calib_report.txt",
        input_name="calib1.mp4",

        resolution=(1640, 1232),
        input_fps=15,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        color_mode=False,
        segmentation_mode=SegmentationMode.INTENSITY,
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
        color_mode=False,
        segmentation_mode=SegmentationMode.SUBPIXEL,
        pos_confidence_th=0.1
    )

    main_config = None
