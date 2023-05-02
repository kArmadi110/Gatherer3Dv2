from core.config_types import Config, InputMode, OutputMode, CalibMode, SegmentationMode, FilterMode


class G3DConstants:
    """Constants for the G3D system.
    With example configurations
    """

    recording_config = Config(
        is_async_mode=True,
        async_q_limit=5,

        input_mode=InputMode.CAMERA,
        output_mode=[OutputMode.VIDEO],
        calib_mode=CalibMode.IMPORT,

        input_folder="./bin/calib1/",
        output_folder="./bin/calib1/images/",
        output_name="calib1",
        input_name="calib1.mp4",

        resolution=(1640, 1232),
        # resolution=(1280, 720),
        input_fps=10,
        input_exposure=100,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        color_mode=False,
        segmentation_mode=SegmentationMode.SUBPIXEL,
        pos_confidence_th=0.1
    )

    scan_config1 = Config(
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
        input_exposure=100000,

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
        calib_mode=CalibMode.CAMERA,

        input_folder="./bin/calib2/",
        output_folder="./bin/calib2/",
        # output_folder="./bin/calib2/intensity",
        # output_folder="./bin/calib1/subpixel",
        # output_folder="./bin/calib1/middle",
        output_name="calib_report",
        input_name="calib2.mp4",

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

        input_mode=InputMode.CAMERA,
        output_mode=[OutputMode.CALIBRATION],  # OutputMode.STREAM
        calib_mode=CalibMode.LASER,

        input_folder="./bin/",
        output_folder="./bin/calib1",
        output_name="test1",
        input_name="test1",

        resolution=(1640, 1232),
        input_fps=30,
        input_exposure=10000,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        # laser_gpio=0,
        color_mode=False,
        segmentation_mode=SegmentationMode.SUBPIXEL,
        pos_confidence_th=0.5,

        # filter_mode=FilterMode.RADIUS,
        # filter_param_1=20,
        # filter_param_2=0.001

        filter_mode=FilterMode.STATISTICAL,
        filter_param_1=10,
        filter_param_2=2.0
    )

    scan_config2 = Config(
        is_async_mode=True,
        async_q_limit=5,

        input_mode=InputMode.CAMERA,
        # output_mode=[OutputMode.MESH, OutputMode.STREAM],
        output_mode=[OutputMode.MESH],
        calib_mode=CalibMode.NONE,

        input_folder="./bin/",
        output_folder="./docs/pcd_original/",
        output_name="xcube",
        input_name="test1",

        resolution=(1640, 1232),
        input_fps=30,
        input_exposure=10000,

        streamer_res=(640, 480),
        streaner_path="rtsp://localhost:8554/G3D",

        laser_gpio=14,
        color_mode=False,
        segmentation_mode=SegmentationMode.SUBPIXEL,
        pos_confidence_th=0.2,
        export_cage=True,

        filter_mode=FilterMode.STATISTICAL,
        filter_param_1=10,
        filter_param_2=2.0
    )

    main_config = None
