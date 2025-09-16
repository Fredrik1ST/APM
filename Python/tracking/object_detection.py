



import pyzed.sl as sl
import ogl_viewer.viewer as gl




def init_object_detection(camera, detection_confidence = 40, show_image = True, detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST):
    # Set initialization parameters
    detection_parameters = sl.ObjectDetectionParameters()
    detection_parameters.enable_tracking = True # Objects will keep the same ID between frames
    detection_parameters.enable_segmentation = True # Outputs 2D masks over detected objects

    # Set runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]
    obj_runtime_param.object_class_detection_confidence_threshold[sl.OBJECT_CLASS.PERSON] = detection_confidence

    # choose a detection model
    detection_parameters.detection_model = detection_model  

    if detection_parameters.enable_tracking :
        # Set positional tracking parameters
        positional_tracking_parameters = sl.PositionalTrackingParameters()
        # Enable positional tracking
        camera.enable_positional_tracking(positional_tracking_parameters)

    # Enable object detection with initialization parameters
    zed_error = camera.enable_object_detection(detection_parameters)
    if zed_error != sl.ERROR_CODE.SUCCESS:
        print("enable_object_detection", zed_error, "\nExit program.")
        camera.close()
        exit(-1)

    # Viewer:
    if show_image:
        camera_info = camera.get_camera_information()
        viewer = gl.GLViewer()
        viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam, detection_parameters.enable_tracking)
        print("init viewer")
    else:
        viewer = None

    objects = sl.Objects() # Structure containing all the detected objects
    return viewer, objects, obj_runtime_param

