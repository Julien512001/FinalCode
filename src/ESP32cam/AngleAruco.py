import cv2
import urllib.request
import numpy as np
import cv2.aruco as aruco

def angle_trigo(square_corners):
    delta_x = (square_corners[0][0]+square_corners[3][0])/2 - (square_corners[1][0]+square_corners[2][0])/2
    delta_y = (square_corners[0][1]+square_corners[3][1])/2 - (square_corners[1][1]+square_corners[2][1])/2
    angle = np.arctan2(delta_y, delta_x)
    angle_degrees = np.degrees(angle)
    
    return angle_degrees

# URL de l'image à télécharger et à traiter
url = 'http://192.168.25.132/cam-hi.jpg'

# Télécharge l'image depuis l'URL
img_resp = urllib.request.urlopen(url)
img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)

im = cv2.imdecode(img_np, -1)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(im, aruco_dict, parameters=parameters)

if ids is not None:
    for i in range(len(ids)):
        if ids[i] == 47:
            square_corners = corners[i][0]
            angle = angle_trigo(square_corners)
            with open("angle.txt", "w") as file:
                    file.write(str(round(angle, 2)))

# Affiche l'image avec les marqueurs détectés
# cv2.imshow('ArUco Markers Detection', im)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
