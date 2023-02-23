import rospy
import PySpin
import std_msgs
import vectornav.msg

class TriggerType:
    SOFTWARE = 1
    HARDWARE = 2

CHOSEN_TRIGGER = TriggerType.HARDWARE

class ChunkDataTypes:
    IMAGE = 1
    NODEMAP = 2

CHOSEN_CHUNK_DATA_TYPE = ChunkDataTypes.IMAGE


def configure_exposure(cam):
    """
     This function configures a custom exposure time. Automatic exposure is turned
     off in order to allow for the customization, and then the custom setting is
     applied.

     :param cam: Camera to configure exposure for.
     :type cam: CameraPtr
     :return: True if successful, False otherwise.
     :rtype: bool
    """

    print('*** CONFIGURING EXPOSURE ***\n')

    try:
        result = True
        if cam.ExposureAuto.GetAccessMode() != PySpin.RW:
            print('Unable to disable automatic exposure. Aborting...')
            return False

        cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
        print('Automatic exposure disabled...')

        if cam.ExposureTime.GetAccessMode() != PySpin.RW:
            print('Unable to set exposure time. Aborting...')
            return False

        # Ensure desired exposure time does not exceed the maximum
        cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Continuous)
        print('Camera Exposure set to Auto')

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result

def configure_chunk_data(nodemap):
    """
    This function configures the camera to add chunk data to each image. It does
    this by enabling each type of chunk data before enabling chunk data mode.
    When chunk data is turned on, the data is made available in both the nodemap
    and each image.

    :param nodemap: Transport layer device nodemap.
    :type nodemap: INodeMap
    :return: True if successful, False otherwise
    :rtype: bool

    """
    try:
        result = True
        print('\n*** CONFIGURING CHUNK DATA ***\n')
        chunk_mode_active = PySpin.CBooleanPtr(nodemap.GetNode('ChunkModeActive'))

        if PySpin.IsWritable(chunk_mode_active):
            chunk_mode_active.SetValue(True)

        print('Chunk mode activated...')

        chunk_selector = PySpin.CEnumerationPtr(nodemap.GetNode('ChunkSelector'))

        if not PySpin.IsReadable(chunk_selector) or not PySpin.IsWritable(chunk_selector):
            print('Unable to retrieve chunk selector. Aborting...\n')
            return False
        entries = [PySpin.CEnumEntryPtr(chunk_selector_entry) for chunk_selector_entry in chunk_selector.GetEntries()]

        print('Enabling entries...')

        # Iterate through our list and select each entry node to enable
        for chunk_selector_entry in entries:
            # Go to next node if problem occurs
            if not PySpin.IsReadable(chunk_selector_entry):
                continue

            chunk_selector.SetIntValue(chunk_selector_entry.GetValue())

            chunk_str = '\t {}:'.format(chunk_selector_entry.GetSymbolic())

            # Retrieve corresponding boolean
            chunk_enable = PySpin.CBooleanPtr(nodemap.GetNode('ChunkEnable'))

            # Enable the boolean, thus enabling the corresponding chunk data
            if not PySpin.IsAvailable(chunk_enable):
                print('{} not available'.format(chunk_str))
                result = False
            elif chunk_enable.GetValue() is True:
                print('{} enabled'.format(chunk_str))
            elif PySpin.IsWritable(chunk_enable):
                chunk_enable.SetValue(True)
                print('{} enabled'.format(chunk_str))
            else:
                print('{} not writable'.format(chunk_str))
                result = False

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result


def configure_trigger(cam):
    """
    This function configures the camera to use a trigger. First, trigger mode is
    set to off in order to select the trigger source. Once the trigger source
    has been selected, trigger mode is then enabled, which has the camera
    capture only a single image upon the execution of the chosen trigger.

     :param cam: Camera to configure trigger for.
     :type cam: CameraPtr
     :return: True if successful, False otherwise.
     :rtype: bool
    """
    result = True

    print('*** CONFIGURING TRIGGER ***\n')

    print('Note that if the application / user software triggers faster than frame time, the trigger may be dropped / skipped by the camera.\n')
    print('If several frames are needed per trigger, a more reliable alternative for such case, is to use the multi-frame mode.\n\n')

    if CHOSEN_TRIGGER == TriggerType.SOFTWARE:
        print('Software trigger chosen ...')
    elif CHOSEN_TRIGGER == TriggerType.HARDWARE:
        print('Hardware trigger chose ...')

    try:
        # Ensure trigger mode off
        # The trigger must be disabled in order to configure whether the source
        # is software or hardware.
        nodemap = cam.GetNodeMap()
        node_trigger_mode = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerMode'))
        if not PySpin.IsReadable(node_trigger_mode) or not PySpin.IsWritable(node_trigger_mode):
            print('Unable to disable trigger mode (node retrieval). Aborting...')
            return False

        node_trigger_mode_off = node_trigger_mode.GetEntryByName('Off')
        if not PySpin.IsReadable(node_trigger_mode_off):
            print('Unable to disable trigger mode (enum entry retrieval). Aborting...')
            return False

        node_trigger_mode.SetIntValue(node_trigger_mode_off.GetValue())

        print('Trigger mode disabled...')

        # Set TriggerSelector to FrameStart
        # For this example, the trigger selector should be set to frame start.
        # This is the default for most cameras.
        node_trigger_selector= PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSelector'))
        if not PySpin.IsReadable(node_trigger_selector) or not PySpin.IsWritable(node_trigger_selector):
            print('Unable to get trigger selector (node retrieval). Aborting...')
            return False

        node_trigger_selector_framestart = node_trigger_selector.GetEntryByName('FrameStart')
        if not PySpin.IsReadable(node_trigger_selector_framestart):
            print('Unable to set trigger selector (enum entry retrieval). Aborting...')
            return False
        node_trigger_selector.SetIntValue(node_trigger_selector_framestart.GetValue())

        print('Trigger selector set to frame start...')

        # Select trigger source
        # The trigger source must be set to hardware or software while trigger
        # mode is off.
        node_trigger_source = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource'))
        if not PySpin.IsReadable(node_trigger_source) or not PySpin.IsWritable(node_trigger_source):
            print('Unable to get trigger source (node retrieval). Aborting...')
            return False

        if CHOSEN_TRIGGER == TriggerType.SOFTWARE:
            node_trigger_source_software = node_trigger_source.GetEntryByName('Software')
            if not PySpin.IsReadable(node_trigger_source_software):
                print('Unable to get trigger source (enum entry retrieval). Aborting...')
                return False
            node_trigger_source.SetIntValue(node_trigger_source_software.GetValue())
            print('Trigger source set to software...')

        elif CHOSEN_TRIGGER == TriggerType.HARDWARE:
            node_trigger_source_hardware = node_trigger_source.GetEntryByName('Line0')
            if not PySpin.IsReadable(node_trigger_source_hardware):
                print('Unable to get trigger source (enum entry retrieval). Aborting...')
                return False
            node_trigger_source.SetIntValue(node_trigger_source_hardware.GetValue())
            print('Trigger source set to hardware...')

        # Turn trigger mode on
        # Once the appropriate trigger source has been set, turn trigger mode
        # on in order to retrieve images using the trigger.
        node_trigger_mode_on = node_trigger_mode.GetEntryByName('On')
        if not PySpin.IsReadable(node_trigger_mode_on):
            print('Unable to enable trigger mode (enum entry retrieval). Aborting...')
            return False

        node_trigger_mode.SetIntValue(node_trigger_mode_on.GetValue())
        print('Trigger mode turned back on...')

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return False

    return result


def set_acquisition_mode(cam_list):

    try:
        result = True

        # Prepare each camera to acquire images
        #
        # *** NOTES ***
        # For pseudo-simultaneous streaming, each camera is prepared as if it
        # were just one, but in a loop. Notice that cameras are selected with
        # an index. We demonstrate pseduo-simultaneous streaming because true
        # simultaneous streaming would require multiple process or threads,
        # which is too complex for an example.
        #

        for i, cam in enumerate(cam_list):

            # Set acquisition mode to continuous
            node_acquisition_mode = PySpin.CEnumerationPtr(cam.GetNodeMap().GetNode('AcquisitionMode'))
            if not PySpin.IsReadable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
                print('Unable to set acquisition mode to continuous (node retrieval; camera %d). Aborting... \n' % i)
                return False

            node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName('Continuous')
            if not PySpin.IsReadable(node_acquisition_mode_continuous):
                print('Unable to set acquisition mode to continuous (entry \'continuous\' retrieval %d). \
                Aborting... \n' % i)
                return False

            acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

            node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

            print('Camera %d acquisition mode set to continuous...' % i)

            # Begin acquiring images
            cam.BeginAcquisition()

            print('Camera %d started acquiring images...' % i)

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        result = False

    return result

def acquire_timestamp(cam):
    try:
        image_result = cam.GetNextImage(50)
        chunk_data = image_result.GetChunkData()
        timestamp = chunk_data.GetTimestamp()
        image_result.Release()

        return timestamp    

    except PySpin.SpinnakerException as ex:
        print('Error: %s' % ex)
        return


rospy.init_node('SyncAnalyzer', anonymous=True)


center_pub = rospy.Publisher("centercam_timedelta", std_msgs.msg.Int64, queue_size=5)

right_pub = rospy.Publisher("rightcam_timedelta", std_msgs.msg.Int64, queue_size=5)

left_pub = rospy.Publisher("leftcam_timedelta", std_msgs.msg.Int64, queue_size=5)

ins_pub = rospy.Publisher("INS_timedelta", std_msgs.msg.Int64, queue_size=5)


def center(cam_list):
    center_old_timestamp = 0
    centercam = cam_list.GetBySerial('17512985')
    while True:        
        #SyncVerify.msg.SyncVerify.old_device_timestamp = center_old_timestamp
        timestamp = acquire_timestamp(centercam)
        #SyncVerify.msg.SyncVerify.current_device_timestamp = timestamp
        time_delta = timestamp - center_old_timestamp
        #SyncVerify.msg.SyncVerify.device_time_delta = time_delta
        center_old_timestamp = timestamp
        center_pub.publish(time_delta)

def left(cam_list):
    left_old_timestamp = 0
    leftcam = cam_list.GetBySerial('18060270')
    while True:        
        #SyncVerify.msg.SyncVerify.old_device_timestamp = left_old_timestamp
        timestamp = acquire_timestamp(leftcam)
        #SyncVerify.msg.SyncVerify.current_device_timestamp = timestamp
        time_delta = timestamp - left_old_timestamp
        #SyncVerify.msg.SyncVerify.device_time_delta = time_delta
        left_old_timestamp = timestamp
        left_pub.publish(time_delta)
    

def right(cam_list):
    right_old_timestamp = 0
    rightcam = cam_list.GetBySerial('17528370')
    while True:        
        #SyncVerify.msg.SyncVerify.old_device_timestamp = right_old_timestamp
        timestamp = acquire_timestamp(rightcam)
        #SyncVerify.msg.SyncVerify.current_device_timestamp = timestamp
        time_delta = timestamp - right_old_timestamp
        #SyncVerify.msg.SyncVerify.device_time_delta = time_delta
        right_old_timestamp = timestamp
        right_pub.publish(time_delta)


def INS(data):    
    INS_old_timestamp = 0
    #SyncVerify.msg.SyncVerify.old_device_timestamp = INS_old_timestamp
    timestamp = data.time
    #SyncVerify.msg.SyncVerify.current_device_timestamp = timestamp
    time_delta = timestamp - INS_old_timestamp
    #SyncVerify.msg.SyncVerify.device_time_delta = time_delta
    INS_old_timestamp = timestamp
    ins_pub.publish(time_delta)


def GetSyncVerify():
    rospy.Subscriber("vectornav/INS", vectornav.msg.Ins, INS)
    rospy.spin()


def __init__():
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()

    try:
        for cam in cam_list:
            cam.Init()
            configure_trigger(cam)
            configure_chunk_data(cam.GetNodeMap())            
            configure_exposure(cam)
            set_acquisition_mode(cam)
        
        GetSyncVerify()
        left(cam_list)
        right(cam_list)
        center(cam_list)
    
    except PySpin.SpinnakerException as ex:
        print('ERROR: %s' % ex)
        return
    
    except rospy.exceptions.ROSInterruptException as err:
        print('ROS_ERROR: %s' % err)
        GetSyncVerify.shutdown()
        left_pub.shutdown()
        right_pub.shutdown()
        center_pub.shutdown()
        for cam in cam_list:
            cam.EndAcquisition()
            cam.DeInit()            
        del cam
        cam_list.Clear()
        system.ReleaseInstance()
        del cam_list
        return
    