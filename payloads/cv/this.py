from imutils import paths
import numpy as np
import imutils
import cv2

def stop_search():
	return(cv2.waitKey(1) == ord('\x1b'))

cap = cv2.VideoCapture(0)

pixels = [[], [], [], []]
output = []

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

		# Biggest Contour
		c = max(cnts, key = cv2.contourArea)

		marker = cv2.minAreaRect(c)

		box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
		box = np.int0(box)
		cv2.drawContours(frame, [box], -1, (0, 0, 255), 2)
		cv2.circle(frame, tuple(box[0]), 5, (0, 0, 0), -1)
		cv2.circle(frame, tuple(box[1]), 5, (0, 0, 0), -1)
		cv2.circle(frame, tuple(box[2]), 5, (0, 0, 0), -1)
		cv2.circle(frame, tuple(box[3]), 5, (0, 0, 0), -1)

	cv2.imshow('Frame', frame)
	#cv2.imshow('Mask', canny)

	if len(pixels[0]) < 50:
		for i in [0, 1, 2, 3]:
			pixels[i].append(box[i])
	else:
		for i in range(len(pixels)):
			j = sum(pixels[i])/len(pixels[i])
			for x in range(len(j)):
				j[x] = round(j[x])
			pixels[i] = j

	if stop_search():
		break

print(pixels)

cap.release()
cv2.destroyAllWindows()
