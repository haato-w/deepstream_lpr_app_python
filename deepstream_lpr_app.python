import sys
sys.path.append('../')
sys.path.append('../../')
#sys.path.append('/opt/nvidia/deepstream/deepstream-5.1/lib')
import locale
import time
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
from common.is_aarch_64 import is_aarch64
import pyds

import configparser
import csv

PGIE_CLASS_ID_VEHICLE = 0
PGIE_CLASS_ID_PERSON = 2

SGIE_CLASS_ID_LPD = 0

PRIMARY_DETECTOR_UID = 1
SECONDARY_DETECTOR_UID = 2
SECONDARY_CLASSIFIER_UID = 3

perf_measure = {'pre_time': 0, 'total_time': 0, 'count': 0}
license_plate_record = []
license_plate_coordinate = []

'''
def secondary_detector_sink_pad_buffer_probe(pad, info):
    print("secondary_detector_pad_buffer_probe is called")

    gst_buffer = info.get_buffer()
    if(not gst_buffer):
        print("Unable to get GstBuffer")
        return
    
    #Retrieve batch metadata from the gst_buffer
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
'''

#osd_sink_pad_buffer_probe will extract metadata received on OSD sink pad
#and update params for drawing rectangle, object information etc.

def osd_sink_pad_buffer_probe(pad, info, u_data):

    vehicle_count = 0
    person_count = 0
    lp_count = 0

    print("\nosd_sink_pad_buffer_probe is called")
    frame_count[0] += 1

    gst_buffer = info.get_buffer()
    if(not gst_buffer):
        print("Unable to get GstBuffer")
        return
    
    #Retrieve batch metadata from the gst_buffer
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))

    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        print("frame_meta_list loop")
        try:
            #Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            #The casting is done by pyds.NvDsFrameMeta.cast()
            #The casting also keeps ownership of the underlying memory
            #in the C code, so the Python gabrage collector wii leave
            #it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break
        offset = 0
        if(not frame_meta):
            continue
        
        l_obj = frame_meta.obj_meta_list
        while l_obj is not None:
            print("obj_meta_list loop")
            try:
                obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break
            if(not obj_meta):
                continue
                
            #Check that the object has been detected by the primary detector
            #and that the class id is that of vehicles/persons.
            if(obj_meta.unique_component_id == PRIMARY_DETECTOR_UID):
                print("primary_detector is working")
                print("vehicle_tracking_id: " + str(obj_meta.object_id))
                if(obj_meta.class_id == PGIE_CLASS_ID_VEHICLE):
                    vehicle_count += 1
                if(obj_meta.class_id == PGIE_CLASS_ID_PERSON):
                    person_count += 1

            if(obj_meta.unique_component_id == SECONDARY_DETECTOR_UID):
                print("secondary_detector is working")
                parent_obj_meta = pyds.NvDsObjectMeta.cast(obj_meta.parent)
                parent_tracking_id = str(parent_obj_meta.object_id)
                print("parent_tracking_id: " + parent_tracking_id)
                if(obj_meta.class_id == SGIE_CLASS_ID_LPD):
                    lp_count += 1
                #print(obj_meta.rect_params)
                rect_params_info = obj_meta.rect_params
                print("rect_params_left: " + str(rect_params_info.left))
                print("rect_params_top: " + str(rect_params_info.top))
                print("rect_params_width: " + str(rect_params_info.width))
                print("rect_params_height: " + str(rect_params_info.height))
                license_plate_coordinate.append([frame_count[0], parent_tracking_id, rect_params_info.left, rect_params_info.top, rect_params_info.width, rect_params_info.height])

            l_class = obj_meta.classifier_meta_list  
            while l_class is not None:
                try:
                    class_meta = pyds.NvDsClassifierMeta.cast(l_class.data)
                except StopIteration:
                    break
                if(not class_meta):
                    continue

                if(class_meta.unique_component_id == SECONDARY_CLASSIFIER_UID):
                    print("secondary_classifier is working")
                    label_i = 0
                    l_label = class_meta.label_info_list
                    while(label_i < class_meta.num_labels and l_label):

                        label_info = pyds.NvDsLabelInfo.cast(l_label.data)
                        if(label_info):
                            if(label_info.label_id == 0 and label_info.result_class_id == 1):
                                result_label = label_info.result_label
                                if(len(license_plate_record)>5):
                                    if not result_label in license_plate_record[-5:]:
                                        license_plate_record.append(result_label)
                                else:
                                    if not result_label in license_plate_record:
                                        license_plate_record.append(result_label)

                                print("Plate License: " + label_info.result_label)

                        label_i += 1
                        l_label = l_label.next

                try:
                    l_class = l_class.next
                except StopIteration:
                    break

            try:
                l_obj = l_obj.next
            except StopIteration:
                break

        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    print("Vehicle Count = {0} Person Count = {1} License Plate Count = {2}"
        .format(vehicle_count, person_count, lp_count))
    return Gst.PadProbeReturn.OK

def bus_call(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("End of Stream")
        loop.quit()
    elif t == Gst.MessageType.WARNING:
        error, debug = message.parse_warning()
        sys.stderr.write("Warning: %s: %s" % (error, debug))
    elif t == Gst.MessageType.ERROR:
        error, debug = message.parse_error()
        sys.stderr.write("Error: %s: %s" % (error, debug))
        loop.quit()

def cb_new_pad(demuxer, pad, src_ele):
    if pad.get_property("template").name_template == "video_%u":
        src_ele_pad = src_ele.get_static_pad("sink")
        pad.link(src_ele_pad)

def set_tracker_properties(nvtracker, config_file_name):
    print("Setting tracker")
    config = configparser.ConfigParser()
    config.read(config_file_name)

    #print(config.sections())
    section = config.sections()
    #print(section[0])
    for k, v in config[section[0]].items():
        #print("{0}={1}".format(k, v))
        if(k == 'tracker-width'):
            nvtracker.set_property(k, int(v))
        elif(k == 'tracker-height'):
            nvtracker.set_property(k, int(v))
        elif(k == 'gpu-id'):
            nvtracker.set_property(k, int(v))
        elif(k == 'll-lib-file'):
            nvtracker.set_property(k, v)
        elif(k == 'll-config-file'):
            nvtracker.set_property(k, v)
        elif(k == 'enable-batch-process'):
            nvtracker.set_property(k, int(v))
    return True

def main(args):

    #Check input arguments
    print("len(args): " + str(len(args)))
    if(len(args) != 3):
       print("Usage: " + args[0] + "<In mp4 filename> <out H264 filename>")
       return -1
    
    locale.setlocale(locale.LC_CTYPE, "")
    
    #Standard GStreamer Initialized
    GObject.threads_init()
    loop = GObject.MainLoop()
    Gst.init(None)

    #Create gstreamer elements
    #Create Pipeline element that will form a connection of other elements
    pipeline = Gst.Pipeline()

    streammux = Gst.ElementFactory.make("nvstreammux", "stream-muxer")

    if(not pipeline or not streammux):
        print("One element could not be created. Exiting.")
        return -1
    
    pipeline.add(streammux)

    #Source file
    #Only h264 element stream with mp4 container is supported.
    #Source element for reading from the file
    source = Gst.ElementFactory.make("filesrc", "file_src")

    mp4demux = Gst.ElementFactory.make("qtdemux", "mp4demux")

    h264parser = Gst.ElementFactory.make("h264parse", "h264parser")

    parsequeue = Gst.ElementFactory.make("queue", "parsequeue")

    #Use nvdec_h264 for hardware accelerated decode on GPU
    decoder = Gst.ElementFactory.make("nvv4l2decoder", "decoder")

    if(not source or not mp4demux or not h264parser or not parsequeue or not decoder):
        print("One element could not be created. Exiting.")
        return -1

    pipeline.add(source)
    pipeline.add(mp4demux)
    pipeline.add(h264parser)
    pipeline.add(parsequeue)
    pipeline.add(decoder)

    pad_name_sink = "sink_0"
    sinkpad = streammux.get_request_pad(pad_name_sink)
    print("Request " + pad_name_sink + " pad from streammux")
    if(not sinkpad):
        print("Streammux request sink pad failed. Exiting.")
        return -1
    
    pad_name_src = "src"
    srcpad = decoder.get_static_pad(pad_name_src)
    if(not srcpad):
        print("Filed to link decoder to stream muxer.Exiting.")
        return -1
    
    srcpad.link(sinkpad)
    source.link(mp4demux)

    mp4demux.connect("pad-added", cb_new_pad, h264parser)

    h264parser.link(parsequeue)
    parsequeue.link(decoder)

    #we set the input filename to the source element
    source.set_property('location', args[1])

    #Create three nvinfer instances for two detectors and one classifier
    primary_detector = Gst.ElementFactory.make("nvinfer", "primary-infer-engine1")

    secondary_detector = Gst.ElementFactory.make("nvinfer", "secondary-infer-engine1")

    secondary_classifier = Gst.ElementFactory.make("nvinfer", "secondary-infer-engine2")

    #Use converter to convert from NV12 to RGBA as required be nvosd
    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "nvvid-converter")

    #Create OSD to draw on the converter RGBA buffer
    nvosd = Gst.ElementFactory.make("nvdsosd", "nv-onscreendisplay")

    nvvidconv1 = Gst.ElementFactory.make("nvvideoconvert", "nvvid-converter1")

    nvh264enc = Gst.ElementFactory.make("nvv4l2h264enc", "nvvideo-h264enc")

    capfilt = Gst.ElementFactory.make("capsfilter", "nvvideo-caps")

    nvtile = Gst.ElementFactory.make("nvmultistreamtiler", "nvtile")

    tracker = Gst.ElementFactory.make("nvtracker", "nvtracker")

    queue1 = Gst.ElementFactory.make("queue", "queue1")
    queue2 = Gst.ElementFactory.make("queue", "queue2")
    queue3 = Gst.ElementFactory.make("queue", "queue3")
    queue4 = Gst.ElementFactory.make("queue", "queue4")
    queue5 = Gst.ElementFactory.make("queue", "queue5")
    queue6 = Gst.ElementFactory.make("queue", "queue6")
    queue7 = Gst.ElementFactory.make("queue", "queue7")
    queue8 = Gst.ElementFactory.make("queue", "queue8")
    queue9 = Gst.ElementFactory.make("queue", "queue9")
    queue10 = Gst.ElementFactory.make("queue", "queue10")

    sink = Gst.ElementFactory.make("filesink", "nvvideo-renderer")
    
    if(not primary_detector or not secondary_detector or not nvvidconv 
       or not nvosd or not sink or not capfilt or not nvh264enc):
       print("One element could not be crated. Exiting.")
       return -1

    streammux.set_property('width', 1280)
    streammux.set_property('height', 720)
    streammux.set_property('batch-size', 1)
    streammux.set_property('batched-push-timeout', 4000000)

    nvtile.set_property('rows', 1)
    nvtile.set_property('columns', 1)
    nvtile.set_property('width', 1280)
    nvtile.set_property('height', 720)

    #Set the config files for the two detectors and one classifier. The PGIE
    #detects the cars. The first SGIE detects car plates from the cars and the
    #second SGIE classifies the caracters in the car plate to identify the car
    #plat string.
    print("Setting primary detector")
    primary_detector.set_property('config-file-path', 'trafficamnet_config.txt')
    primary_detector.set_property('unique-id', PRIMARY_DETECTOR_UID)

    print("Setting secondary detector")
    secondary_detector.set_property('config-file-path', 'lpd_us_config.txt')
    secondary_detector.set_property('unique-id', SECONDARY_DETECTOR_UID)
    secondary_detector.set_property('process-mode', 2)

    print("Setting secondary classifier")
    secondary_classifier.set_property('config-file-path', 'lpr_config_sgie_us.txt')
    secondary_classifier.set_property('unique-id', SECONDARY_CLASSIFIER_UID)
    secondary_classifier.set_property('process-mode', 2)

    name = 'lpr_sample_tracker_config.txt'
    if(not set_tracker_properties(tracker, name)):
        print("Failed to set tracker1 properties. Exiting.")
        return -1
    
    caps = Gst.Caps.from_string("memory:NVMM, video/x-raw, format, G_TYPE_STRING, I420")
    #caps.set_features(0, feature)
    capfilt.set_property("caps", caps)

    #we add a bus message handler
    bus  = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)

    #Set up the pipeline
    #we add all elements into the pipeline
    pipeline.add(primary_detector)
    pipeline.add(secondary_detector)
    pipeline.add(tracker)
    pipeline.add(queue1)
    pipeline.add(queue2)
    pipeline.add(queue3)
    pipeline.add(queue4)
    pipeline.add(queue5)
    pipeline.add(queue6)
    pipeline.add(queue7)
    pipeline.add(queue8)
    pipeline.add(secondary_classifier)
    pipeline.add(nvvidconv)
    pipeline.add(nvosd)
    pipeline.add(nvtile)
    pipeline.add(sink)

    streammux.link(queue1)
    queue1.link(primary_detector)
    primary_detector.link(queue2)
    queue2.link(tracker)
    tracker.link(queue3)
    queue3.link(secondary_detector)
    secondary_detector.link(queue5)
    queue5.link(secondary_classifier)
    secondary_classifier.link(queue6)
    queue6.link(nvtile)
    nvtile.link(queue7)
    queue7.link(nvvidconv)
    nvvidconv.link(queue8)
    queue8.link(nvosd)

    sink.set_property("location", args[len(args)-1] + ".264")
    pipeline.add(nvvidconv1)
    pipeline.add(nvh264enc)
    pipeline.add(capfilt)
    pipeline.add(queue9)
    pipeline.add(queue10)

    nvosd.link(queue9)
    queue9.link(nvvidconv1)
    nvvidconv1.link(capfilt)
    capfilt.link(queue10)
    queue10.link(nvh264enc)
    nvh264enc.link(sink)

    #Lets add probe to get informed of the meta data generated, we add probe to 
    #the sink pad of the osd element, since by that time, the buffer would have 
    #had got all the metadata.
    osd_sink_pad = nvosd.get_static_pad("sink")
    if(not osd_sink_pad):
        print("Unable to get sink pad")
    else:
        osd_sink_pad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 
                               perf_measure)
    
    '''
    secondary_detector_sink_pad = secondary_detector.get_static_pad("sink")
    if(not secondary_detector_sink_pad):
        print("Unable to get sink pad")
    else:
        secondary_detector_sink_pad.add_probe(Gst.PadProbeType.BUFFER, secondary_detector_sink_pad_buffer_probe)
    '''

    #Set the pipeline to "playing" state
    print("Now playing: " + args[0])
    pipeline.set_state(Gst.State.PLAYING)

    #Wait  till pipeline encounters an error or EOS
    print("Running...")
    try:
        loop.run()
    except:
        pass

    #Out of the main loop, clean up nicely
    print("Returned, stopping playback")
    pipeline.set_state(Gst.State.NULL)

    #print("Average fps %f")
    #print("Totally %d plates are inferred")
    print("License_plate_record: ")
    print(license_plate_record)
    print("\nLicense_plate_coordinate")
    print(license_plate_coordinate)
    print("\nframe_num: " + str(frame_count[0]))

    with open(args[len(args)-1] + '.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerows(license_plate_coordinate)


if __name__ == '__main__':
    frame_count = [0]
    sys.exit(main(sys.argv))