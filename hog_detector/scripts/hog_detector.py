#!/usr/bin/env python2
# 
# Este Script se suscribe 
#
# Autor: Ernesto Martín Jiménez - asedifusco@correo.ugr.es

# Importamos las librerias necesarias
import rospy 							# Libreria principal de ROS para Python
from sensor_msgs.msg import Image 				# El mensaje de tipo imagen de ROS
from geometry_msgs.msg import PoseWithCovarianceStamped 	# El mensaje para la posiciñón del robot
from cv_bridge import CvBridge				        # Paquete para transformar las imagenes desde ROS a OpenCV
import cv2 							# Librería OpenCV
import numpy as np 						# Paquete para trabajar con arrays 
from imutils.object_detection import non_max_suppression	# Paquete para realizar supresion de no maximos
from imutils import paths
import imutils
import time							# Paquete para medir el tiempo
import centroidtracker						# Paquete para calcular y trackear los centroides
from centroidtracker import CentroidTracker

# Definiciones globales ###############################################

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
ct = CentroidTracker()
len2 = 0
posicion = [0,0,0]
(H, W) = (None, None)

#######################################################################

def callback(data):
 
  global len2
  global posicion
  global H
  global W
  global CONFIDENCE
  
  # Umbral definido para desechar detecciones malas
  umbral = 1
  
  # Para convertir imagenes de ROS a OpenCV. Se ajusta el tamaño y se cambia a escala de grises para velocidad de detección
  br = CvBridge()
  frame = br.imgmsg_to_cv2(data)
  frame = imutils.resize(frame, width=min(400, frame.shape[1]))
  orig = frame.copy()
  gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
  
  if W is None or H is None:
    (H, W) = frame.shape[:2]

  # Se toma la primera medida de reloj para calcular el tiempo de cada detección
  t1 = time.clock()

  # Detección multiescala mediante HOG
  boxes, weights = hog.detectMultiScale(frame, winStride=(4, 4), padding=(8, 8), scale=1.2)

  # Después de cada detección se toma el timpo de nuevo
  t2 = time.clock()

  boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
  pick = boxes
  
  # Se recorren las detecciones para ver si las incluimos o no, cuando se incluyen se hace la supresión de no maximos. Tambien se muestra la posición en la que se ha detectado la persona
  if len(weights) == 1:
    if weights > umbral:
      objects = ct.update(pick)
      pick = non_max_suppression(boxes, probs=None, overlapThresh=0.65)
      print("Persona detectada en posicion:", posicion)
      for (x, y, w, h) in boxes:
        cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
  
      for (xA, yA, xB, yB) in pick:
        cv2.rectangle(frame, (xA, yA), (xB, yB),(0, 255, 0), 2)

      for (objectID, centroid) in objects.items():
        text = "ID {}".format(objectID)
        cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
  else:
    for pesos in weights:
      if pesos > umbral:
        pos = np.where(weights == pesos)[0]
        objects = ct.update(pick[pos,:])
        print("Persona detectada en posicion:", posicion)
        pick = non_max_suppression(boxes, probs=None, overlapThresh=0.65)
        for (x, y, w, h) in boxes:
          cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

        for (xA, yA, xB, yB) in pick:
          cv2.rectangle(frame, (xA, yA), (xB, yB),(0, 255, 0), 2)

        for (objectID, centroid) in objects.items():
          text = "ID {}".format(objectID)
          cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
          cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
  
  
  #Se muestra la imagen de salida
  cv2.imshow("Frame", frame)
  key = cv2.waitKey(4) & 0xFF

  cv2.waitKey(10)
      
def receive_message():
 
  # Para suscribirse al topic de video y de localización
  rospy.init_node('video_sub_py', anonymous=True)
   
  rospy.Subscriber('/camera/color/image_raw', Image, callback)
  rospy.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped ,callback2)
  
  rospy.spin()
 
  # Cerrar todas las ventanas al parar
  cv2.destroyAllWindows()

def callback2(data):
  global posicion
  posicion = data.pose.pose.position
  

if __name__ == '__main__':
    receive_message()
