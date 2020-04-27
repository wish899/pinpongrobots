##Part of CS 498 IoT Project

import io
import os
import sys
import socket
import cv2
import numpy as np
import threading
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import struct


import time

from PIL import Image



led_on = "0"
distance = "0"

def detection():

    global led_on
    global distance

    # Resolution Constants
    IM_WIDTH = 640
    IM_HEIGHT = 480


    stream = io.BytesIO() #get bytes stream 


    client_socket = socket.socket()
    client_socket.connect(('10.0.0.5', 8000))  # IP address needs to be put here
    connection = client_socket.makefile('wb')


    sys.path.append('/opt/models/research/')


    from object_detection.utils import label_map_util
    from object_detection.utils import visualization_utils as vis_util


    MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

    BASE_PATH = '/opt/models/research/object_detection'

    FROZEN_PATH = os.path.join(BASE_PATH,MODEL_NAME,'frozen_inference_graph.pb')


    LABELS_PATH = os.path.join(BASE_PATH,'data','mscoco_label_map.pbtxt')


    NUM_CLASSES = 90 # from ssdmobilenet class

    #Load labels and categories
    label_map = label_map_util.load_labelmap(LABELS_PATH)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    # Load tf model into memory and create session.
    detection_graph = tf.compat.v1.Graph()
    with detection_graph.as_default():
        obj_graph_def = tf.compat.v1.GraphDef()
        with tf.io.gfile.GFile(FROZEN_PATH, 'rb') as fid:
            serialized_graph = fid.read()
            obj_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(obj_graph_def, name='')

        sess = tf.compat.v1.Session(graph=detection_graph)



    #Get image input tensor
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    #Get the output tensors: boxes, scores and classes
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

    #Get the number of detections we get per run
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Initialize frame rate and font
    frame_rate_calc = 10
    font = cv2.FONT_HERSHEY_SIMPLEX


    #Initialize camera
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 10
    rawCap = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCap.truncate(0) 

    try:
        for frame_new in camera.capture_continuous(rawCap, format="bgr",use_video_port=True):

            start = time.time()
            
            #Make frame writeable so that we can draw the bounding boxes on it
            frame = np.copy(frame_new.array)
            frame.setflags(write=1)
            frame_expanded = np.expand_dims(frame, axis=0)

            # Do the detection
            (boxes, scores, classes, num) = sess.run(
                [detection_boxes, detection_scores, detection_classes, num_detections],
                feed_dict={image_tensor: frame_expanded})

        
            vis_util.visualize_boxes_and_labels_on_image_array(
                frame,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                line_thickness=8,
                min_score_thresh=0.40)

            cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
            
            
            # Logic for determining whether to turn LED on: 
            # If the top class found was human with greater than 60% probability 
            # and the distance from the ultrasonic sensor is less than 30 cm, 
            # we turn the LED lights on. 
            if (int(classes[0][0])) == 1 and (float(scores[0][0]) > 0.6):
                cv2.putText(frame, "We found a person!", (100, 100), font, 1, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(frame, "probability: {0:.2f}".format(scores[0][0]), (100, 130), font, 1, (0, 255, 255), 2, cv2.LINE_AA)
                if int(distance) < 30:
                    led_on = "1"
                else:
                    led_on = "0"
            else:
                led_on = "0"

            cv2.putText(frame, "Distance: " + distance, (70,70), font,1, (0, 0, 255), 2, cv2.LINE_AA)

            #Now, let's convert the image into a stream to send through ethernet ...
            
            #Open CV uses BGR, so need to reverse it to RGB for pyplot display
            frame = frame[..., ::-1]
            pil_im = Image.fromarray(frame)
            pil_im.save(stream, format = 'JPEG') #saving in bytes stream to send through ethernet

            connection.write(struct.pack('<L', stream.tell()))
            connection.flush()
            stream.seek(0)
            connection.write(stream.read())
            stream.seek(0)
            stream.truncate()

            #Calculate FPS
            cur = time.time()
            time_t = cur - start
            frame_rate_calc = 1/time_t

            # Press 'q' to escape program
            if cv2.waitKey(1) == ord('q'):
                break

            rawCap.truncate(0)
        connection.write(struct.pack('<L', 0))

    finally:
        connection.close()
        camera.close()
        cv2.destroyAllWindows()

def udp_server():
    global led_on
    global distance
    
    HOST = '0.0.0.0'   # Symbolic name meaning all available interfaces
    PORT = 1234 # Arbitrary non-privileged port

    # Datagram (udp) socket
    try :
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print('Socket created')
    except socket.error as msg :
        print('Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
        sys.exit()


    # Bind socket to local host and port
    try:
        s.bind((HOST, PORT))
    except socket.error as msg:
        print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
        sys.exit()
        
    print('Socket bind complete')

    #now keep talking with the client
    while 1:
        # receive data from client (data, addr)
        d = s.recvfrom(1024)
        data = d[0]
        addr = d[1]
        
        if not data: 
            break
        
        #reply = 'OK...' + data.decode()
        
        s.sendto(led_on.encode() , addr)
        #eply = '0'
        
        print('Message[' + addr[0] + ':' + str(addr[1]) + '] - ' + data.decode().strip())
        distance =  data.decode().strip()
        #if int(distance) < 30:
         #   reply = '1'
        #s.sendto(reply.encode(), addr)
        
    s.close()

def main():

    detection_thread = threading.Thread(target=detection)
    udp_thread = threading.Thread(target=udp_server)
    detection_thread.start()
    udp_thread.start()

if __name__ == '__main__':
    main()