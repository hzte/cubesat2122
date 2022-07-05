from imutils import paths
import numpy as np
import imutils
import cv2

cap = cv2.VideoCapture('http://192.168.4.1:8000/stream.mjpg')


while True:
    ret, frame = cap.read()
    width = int(cap.get(3))
    height = int(cap.get(4))

	#
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    canny =  cv2.Canny(gray, 150, 200)

    # Contours
    cnts = cv2.findContours(canny.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) != 0:

		# Biggest Contourz
        c = max(cnts, key = cv2.contourArea)

        marker = cv2.minAreaRect(c)
        
        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)

        print(box[0], box[1], box[2], box[3])
        cv2.rectangle(frame,(box[0]),(box[2]),(255, 0, 0),3)
        cv2.imshow('Frame', frame)
	#cv2.imshow('Mask', canny)

    if cv2.waitKey(1) == ord('z'):
        break

cap.release()
cv2.destroyAllWindows()
