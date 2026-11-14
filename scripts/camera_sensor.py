#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String  # <-- AGGIUNTO PER IL SEGNALE DI RESET
from cv_bridge import CvBridge, CvBridgeError

class SmartVisionMediator:
    def __init__(self):
        rospy.init_node('smart_vision_mediator', anonymous=True)
        self.bridge = CvBridge()
        
        # --- COSTANTI DI CALIBRAZIONE ---
        self.PPM = 554.25  
        self.TOLLERANZA_PX = 30  
        self.Z_PRESA_SICURA = 0.055 
        
        # --- MEMORIA DI STATO ---
        self.oggetti_sul_tavolo = [] 
        self.ultimo_sparo = None     
        self.colpo_a_vuoto_flag = False
        self.unreachable_flag = False

        # --- CONNESSIONI ROS ---
        self.image_sub = rospy.Subscriber("/overhead_camera/image_raw", Image, self.camera_callback)
        self.laser_sub = rospy.Subscriber("/bersaglio_laser_grezzo", Point, self.laser_callback)
        self.status_sub = rospy.Subscriber("/vision/target_status", String, self.status_callback)

        self.image_pub = rospy.Publisher("/vision/interfaccia_hud", Image, queue_size=1)
        self.robot_target_pub = rospy.Publisher("/target_presa_robot", Point, queue_size=10) 
        
        # Publisher per sbloccare il laser se fallisce il colpo
        self.laser_reset_pub = rospy.Publisher("/bersaglio_laser_fallito", String, queue_size=1)
        
        rospy.loginfo("FASE 3: Mediatore Intelligente AVVIATO. Aspetto il laser...")

    def gazebo_to_pixel(self, x, y):
        u = int(320 - (y * self.PPM))
        v = int(240 - ((x - 0.35) * self.PPM))
        return (u, v)

    def pixel_to_gazebo(self, u, v):
        x = 0.35 - ((v - 240) / self.PPM)
        y = 0.0 - ((u - 320) / self.PPM)
        return (x, y)

    def status_callback(self, msg):
        if msg.data == "UNREACHABLE":
            self.unreachable_flag = True
            self.laser_reset_pub.publish("RESET")
        else:
            self.unreachable_flag = False


    def laser_callback(self, msg):
        laser_x_metri = msg.x
        laser_y_metri = msg.y
        
        px_u, px_v = self.gazebo_to_pixel(laser_x_metri, laser_y_metri)
        self.ultimo_sparo = (px_u, px_v)
        
        candidati = []
        for obj in self.oggetti_sul_tavolo:
            contorno = obj["contorno"]
            centro_px = obj["centro"]
            
            distanza = cv2.pointPolygonTest(contorno, (px_u, px_v), measureDist=True)
            if distanza >= -self.TOLLERANZA_PX:
                candidati.append({"distanza": distanza, "centro": centro_px})
        
        # 2. RISOLUZIONE DEL BERSAGLIO
        if len(candidati) == 0:
            rospy.logwarn("COLPO A VUOTO: Nessun oggetto nel raggio di tolleranza.")
            self.colpo_a_vuoto_flag = True
            # Invia il segnale di reset per liberare il codice del laser
            self.laser_reset_pub.publish("RESET")
        else:
            self.colpo_a_vuoto_flag = False
            self.unreachable_flag = False
            candidati.sort(key=lambda item: item["distanza"], reverse=True)
            bersaglio_vincente = candidati[0]["centro"]
            
            target_x, target_y = self.pixel_to_gazebo(bersaglio_vincente[0], bersaglio_vincente[1])
            
            msg_robot = Point()
            msg_robot.x = target_x
            msg_robot.y = target_y
            msg_robot.z = self.Z_PRESA_SICURA 
            
            self.robot_target_pub.publish(msg_robot)
            rospy.loginfo(f"🎯 AGGANCIO EFFETTUATO! Coordinate inviate al robot: X={target_x:.2f}, Y={target_y:.2f}, Z={msg_robot.z:.3f}")

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError: return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        nuovi_oggetti = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 200 < area < 4000:  
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    nuovi_oggetti.append({"centro": (cX, cY), "contorno": cnt})
                    cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 2)
                    cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)

        self.oggetti_sul_tavolo = nuovi_oggetti

        if self.ultimo_sparo is not None:
            cv2.circle(cv_image, self.ultimo_sparo, self.TOLLERANZA_PX, (0, 255, 255), 1)
            cv2.circle(cv_image, self.ultimo_sparo, 3, (0, 255, 255), -1)

            # HUD INFORMATION 
            testo_x = self.ultimo_sparo[0] + self.TOLLERANZA_PX + 5
            testo_y = self.ultimo_sparo[1] + 5
            
            cv2.putText(cv_image, "R = 5.4 cm", (testo_x, testo_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
            
            if self.colpo_a_vuoto_flag:
                cv2.putText(cv_image, "Warning: NO TARGET DETECTED", (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2, cv2.LINE_AA)

            elif self.unreachable_flag:
                cv2.putText(cv_image, "CRITICAL: TARGET UNREACHABLE", (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("Camera sensor", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        SmartVisionMediator()
        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass