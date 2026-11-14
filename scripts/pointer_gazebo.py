#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel

class LaserGazeboCalculator:
    def __init__(self):
        rospy.init_node('laser_pointer_node', anonymous=True)
        
        rospy.Subscriber('/mano_3d', Float32MultiArray, self.callback_dati_mano)
        rospy.Subscriber('/stato_robot', String, self.callback_stato_robot)
        
        # Ascolta se la telecamera dice che il colpo è andato a vuoto
        rospy.Subscriber('/bersaglio_laser_fallito', String, self.callback_reset_laser)
        
        self.pub_target = rospy.Publisher('/bersaglio_laser_grezzo', Point, queue_size=10)
        self.pub_gazebo_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        
        self.altezza_tavolo = 0.20  
        self.nome_sfera = "punto_rosso_laser"
        
        self.robot_in_azione = False
        self.punto_congelato = None
        self.bersaglio_inviato = False 
        
        self.crea_sfera_virtuale_gazebo()
        rospy.loginfo("Calcolatore Laser: ATTIVO. Protezione Workspace integrata.")

    # sblocca il laser all'istante se si spara nel vuoto
    def callback_reset_laser(self, msg):
        if msg.data == "RESET":
            self.bersaglio_inviato = False
            self.punto_congelato = None
            rospy.logwarn("🔄 SBLOCCO LASER: Il colpo precedente era a vuoto. Puoi sparare di nuovo!")

    def crea_sfera_virtuale_gazebo(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            sdf_sfera = f"""
            <sdf version='1.6'>
              <model name='{self.nome_sfera}'>
                <static>true</static>
                <link name='link'>
                  <visual name='visual'>
                    <geometry><sphere><radius>0.0075</radius></sphere></geometry>
                    <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
                  </visual>
                </link>
              </model>
            </sdf>
            """
            pos_iniziale = Pose()
            pos_iniziale.position.x = 0.15
            pos_iniziale.position.y = 0.0
            pos_iniziale.position.z = self.altezza_tavolo
            spawn_model(self.nome_sfera, sdf_sfera, "", pos_iniziale, "world")
        except rospy.ServiceException as e:
            rospy.logerr(f"Errore caricamento sfera: {e}")

    def calcola_intersezione(self, A, B):
        vettore_v = B - A
        if vettore_v[2] >= 0: return None 
        t = (self.altezza_tavolo - A[2]) / vettore_v[2]
        I_x = A[0] + (vettore_v[0] * t)
        I_y = A[1] + (vettore_v[1] * t)
        return np.array([I_x, I_y, self.altezza_tavolo])

    def sposta_sfera_gazebo(self, posizione):
        stato = ModelState()
        stato.model_name = self.nome_sfera
        stato.pose.position.x = posizione[0]
        stato.pose.position.y = posizione[1]
        stato.pose.position.z = posizione[2] 
        stato.reference_frame = "world"
        self.pub_gazebo_state.publish(stato)

    def callback_stato_robot(self, msg):
        if msg.data == "HOME":
            self.robot_in_azione = False
            self.bersaglio_inviato = False 
            self.punto_congelato = None
        else:
            self.robot_in_azione = True

    def callback_dati_mano(self, msg):
        if len(msg.data) < 7: return
        
        if self.bersaglio_inviato and self.punto_congelato is not None:
            self.sposta_sfera_gazebo(self.punto_congelato)
            return

        A = np.array([msg.data[0], msg.data[1], msg.data[2]])
        B = np.array([msg.data[3], msg.data[4], msg.data[5]])
        trigger = msg.data[6]
        
        intersezione = self.calcola_intersezione(A, B)
        
        if intersezione is not None:
            self.sposta_sfera_gazebo(intersezione)
            
            if trigger == 1.0 and not self.bersaglio_inviato:
                raggio = math.sqrt(intersezione[0]**2 + intersezione[1]**2)
                
                if intersezione[0] < 0.05 or raggio > 0.45:
                    rospy.logwarn_throttle(1.5, f"SPARO RIFIUTATO: X={intersezione[0]:.2f}, Y={intersezione[1]:.2f} fuori portata!")
                    return 
                
                self.bersaglio_inviato = True
                self.punto_congelato = intersezione 
                
                target_msg = Point(intersezione[0], intersezione[1], intersezione[2])
                self.pub_target.publish(target_msg)
                
                rospy.loginfo(f"BERSAGLIO VALIDO CONGELATO: X={intersezione[0]:.2f}, Y={intersezione[1]:.2f}")
        else:
            self.sposta_sfera_gazebo(np.array([0.15, 0.0, -0.5]))

if __name__ == '__main__':
    try:
        calc = LaserGazeboCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass