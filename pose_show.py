import cv2
import cv2.aruco as aruco
import math
import numpy as np
import mediapipe as mp
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils 

DESIRED_HEIGHT = 480
DESIRED_WIDTH = 480

def show_estimated_pose(image):
  with mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.5) as pose:
      # Convert the BGR image to RGB and process it with MediaPipe Pose.
    results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    if not results.pose_landmarks:
      return None
    print(results.pose_landmarks)
    annotated_image = image.copy()
    mp_drawing.draw_landmarks(
        annotated_image,
        results.pose_landmarks,
        mp_pose.POSE_CONNECTIONS)
    return annotated_image

#https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
def show_aruco_tags(img):
  image = img.copy()
  arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
  arucoParams = cv2.aruco.DetectorParameters_create()
  (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)k
  if len(corners)>0:
    # flatten the ArUco IDs list
    ids = ids.flatten()
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
      # extract the marker corners (which are always returned in
      # top-left, top-right, bottom-right, and bottom-left order)
      corners = markerCorner.reshape((4, 2))
      (topLeft, topRight, bottomRight, bottomLeft) = corners
      # convert each of the (x, y)-coordinate pairs to integers
      topRight = (int(topRight[0]), int(topRight[1]))
      bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
      bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
      topLeft = (int(topLeft[0]), int(topLeft[1]))
      # draw the bounding box of the ArUCo detection
      cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
      cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
      cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
      cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
      # compute and draw the center (x, y)-coordinates of the ArUco
      # marker
      cX = int((topLeft[0] + bottomRight[0]) / 2.0)
      cY = int((topLeft[1] + bottomRight[1]) / 2.0)
      cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
      # draw the ArUco marker ID on the image
      cv2.putText(image, str(markerID),
        (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)
  return image
        
if __name__ == '__main__':

    imgs = ['images/'+str(num)+'_color.png' for num in range (1,4)]
    images = {name: cv2.imread(name) for name in imgs}
    for name, image in images.items():
      print(name)   
      cv2.imshow("Image",image)
      cv2.waitKey(0)

      #pose landmarks are w.r.t bottom left corner
      est = show_estimated_pose(image)
      cv2.imshow("Image",est)

      cv2.waitKey(0)
      aru = show_aruco_tags(image)
      cv2.imshow("Image", aru)

      cv2.waitKey(0)