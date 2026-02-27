#Use python virtual environment and pip to install cv2 and pupil-apriltags, pupil-apriltags cannot be installed any other way
#start the virtual environment before running, type: (source .venv/bin/activate) to activate virtual environment.


import cv2
from pupil_apriltags import Detector
import numpy


#open camera
cap = cv2.VideoCapture(0)

#detect apriltags
detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)


#detect apriltag
while True:
    ret, frame = cap.read()
    if not ret:
        print("problem w/ grabbing frame!!\n")
        break

    grayscale = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    detections = detector.detect(grayscale)

       #draw results onto frame
    for detection in detections:
        #draw corners
        corners = detection.corners.astype(numpy.int32)
        cv2.polylines(frame, [corners], True, (0,255,0), 2)

        #draw center
        center = detection.center.astype(int)
        cv2.circle(frame, tuple(center), 5, (0,0,255),-1)
        
        #draw tag id
        tag_id = str(detection.tag_id)
        cv2.putText(frame,tag_id, (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)


    # display frame
    cv2.imshow("Apriltag, detection", frame)

    #exit program if q is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


#release
cap.release()
cv2.destroyAllWindows()



