#!/usr/bin/env python3
"""
sensor_3D.py — Nodo ROS per rilevamento gesti mano
Pubblica su:
  /mano_3d          (Float32MultiArray[7]: Ax,Ay,Az,Bx,By,Bz,trigger_pistola)
  /gesto_emergenza  (String: "STOP" quando viene rilevato il PALMO APERTO)
  /riprendi_robot   (String: "VAI" quando l'utente preme INVIO sul terminale)

Gesti riconosciuti:
  "PISTOLA"       → indice + pollice estesi, altre 3 dita chiuse → trigger=1.0
  "PALMO APERTO"  → tutte le dita distese (richiede mano leggermente inclinata) → EMERGENZA
"""

import threading
import math
import numpy as np
import rospy
import cv2
import mediapipe as mp
from std_msgs.msg import Float32MultiArray, String


# ─────────────────────────────────────────────────────────────────────
#  FILTRO ANTI-TREMORE (media mobile)
# ─────────────────────────────────────────────────────────────────────
_memoria_punti: list = []
_LUNGHEZZA_MEMORIA = 5
_robot_in_arresto = False
_stop_gia_pubblicato = False

def _applica_filtro(nuovo_punto: np.ndarray) -> np.ndarray:
    _memoria_punti.append(nuovo_punto)
    if len(_memoria_punti) > _LUNGHEZZA_MEMORIA:
        _memoria_punti.pop(0)
    return np.mean(_memoria_punti, axis=0)


# ─────────────────────────────────────────────────────────────────────
#  THREAD TASTIERA
# ─────────────────────────────────────────────────────────────────────
def _thread_tastiera(pub_riprendi: rospy.Publisher):
    global _robot_in_arresto
    rospy.loginfo("[TASTIERA-SENSOR] Attivo. Premi INVIO per riprendere il robot dopo l'emergenza.")
    while not rospy.is_shutdown():
        try:
            input()   # blocca finché l'utente non preme INVIO
            _robot_in_arresto = False # RESETTIAMO LO STATO GRAFICO
            pub_riprendi.publish(String(data="VAI"))
            rospy.loginfo("[TASTIERA-SENSOR] INVIO → 'VAI' inviato al controller")
        except EOFError:
            break
        except KeyboardInterrupt:
            break


# ─────────────────────────────────────────────────────────────────────
#  NODO PRINCIPALE
# ─────────────────────────────────────────────────────────────────────
def sensore_3d_node():
    rospy.init_node('sensor_3D_node', anonymous=True)
    global _robot_in_arresto, _stop_gia_pubblicato
    
    # Publisher
    pub_mano = rospy.Publisher('/mano_3d', Float32MultiArray, queue_size=10)
    pub_emergenza = rospy.Publisher('/gesto_emergenza', String, queue_size=1, latch=True)
    pub_riprendi  = rospy.Publisher('/riprendi_robot',  String, queue_size=1)

    # Thread tastiera in background
    t_kbd = threading.Thread(target=_thread_tastiera, args=(pub_riprendi,), daemon=True)
    t_kbd.start()

    # MediaPipe
    mp_hands   = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

    # Telecamera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 15)

    rospy.loginfo("[SENSOR_3D] Telecamera attiva.")
    rospy.loginfo("[SENSOR_3D] Gesti: PISTOLA=pick | PALMO=emergenza | INVIO=riprendi")

    cv2.namedWindow('Sensore 3D', cv2.WINDOW_NORMAL)

    def dist2d(p1, p2) -> float:
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    while not rospy.is_shutdown() and cap.isOpened():
        ok, image = cap.read()
        if not ok:
            break

        image     = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results   = hands.process(image_rgb)

        gesto_corrente = "---"   # testo da mostrare nel frame

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # ── Punti chiave ─────────────────────────────────────
                polso       = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                indice_mcp  = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
                indice_tip  = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                pollice_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP]
                pollice_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                medio_tip   = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                medio_mcp   = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
                anulare_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
                anulare_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP]
                mignolo_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
                mignolo_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]

                dimensione_palmo = dist2d(polso, indice_mcp)
                medio_chiuso   = dist2d(medio_tip,   polso) < dist2d(medio_mcp,   polso)
                anulare_chiuso = dist2d(anulare_tip, polso) < dist2d(anulare_mcp, polso)
                mignolo_chiuso = dist2d(mignolo_tip, polso) < dist2d(mignolo_mcp, polso)
                
                # Indice esteso per il puntamento
                indice_esteso  = dist2d(indice_tip,  polso) > dist2d(indice_mcp,  polso) * 1.2

                # Rapporto pollice/palmo
                estensione_pollice = dist2d(pollice_tip, pollice_mcp)
                pollice_alzato     = (estensione_pollice / dimensione_palmo) > 0.80

                dita_laterali_chiuse = medio_chiuso and anulare_chiuso and mignolo_chiuso

                # GESTO PISTOLA
                gesto_pistola = (dita_laterali_chiuse and indice_esteso and pollice_alzato)

                # GESTO PALMO APERTO
                medio_esteso   = dist2d(medio_tip,   polso) > dist2d(medio_mcp,   polso) * 1.2
                anulare_esteso = dist2d(anulare_tip, polso) > dist2d(anulare_mcp, polso) * 1.2
                mignolo_esteso = dist2d(mignolo_tip, polso) > dist2d(mignolo_mcp, polso) * 1.2
                pollice_aperto_alt = (estensione_pollice / dimensione_palmo) > 0.75

                gesto_palmo_aperto = (indice_esteso and medio_esteso and 
                                      anulare_esteso and mignolo_esteso and 
                                      pollice_aperto_alt)

                # ─────────────────────────────────────────────────────
                #  FILTRAGGIO VETTORE DIREZIONALE
                # ─────────────────────────────────────────────────────
                nocca_grezzo = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
                punta_grezzo = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                vettore_grezzo = np.array([
                    nocca_grezzo.x, nocca_grezzo.y, nocca_grezzo.z,
                    punta_grezzo.x, punta_grezzo.y, punta_grezzo.z
                ])
                v = _applica_filtro(vettore_grezzo)

                A_x = 0.15 + (0.5 - v[1]) * 0.5
                A_y = (v[0] - 0.5) * -0.6
                A_z = 0.40
                dx = v[3]-v[0]; dy = v[4]-v[1]; dz = v[5]-v[2]
                B_x = A_x - dy * 0.8
                B_y = A_y - dx * 0.8
                B_z = A_z + dz * 0.8

                trigger = 1.0 if gesto_pistola else 0.0

                # Pubblica vettore mano
                msg       = Float32MultiArray()
                msg.data  = [A_x, A_y, A_z, B_x, B_y, B_z, trigger]
                pub_mano.publish(msg)

                # GESTIONE EMERGENZA 
                if gesto_palmo_aperto:
                    _robot_in_arresto = True  # Attiva lo stato della HUD
                    if not _stop_gia_pubblicato:
                        pub_emergenza.publish(String(data="STOP"))
                        _stop_gia_pubblicato = True
                        rospy.logwarn("[SENSOR_3D] 🛑 EMERGENZA ATTIVATA → STOP inviato!")
                else:
                    _stop_gia_pubblicato = False

                # Assegnazione della stringa HUD basata sullo STATO di blocco
                if _robot_in_arresto:
                    gesto_corrente = "STOP EMERGENZA ATTIVO"
                elif gesto_pistola:
                    gesto_corrente = "PISTOLA - PICK"
                else:
                    gesto_corrente = "---"

        # Scrittura testo Hud 
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (640, 50), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, image, 0.5, 0, image)

        if _robot_in_arresto:
            colore = (0, 0, 255) 
            testo_hud = "STATUS: STOP EMERGENCY"
        elif "GUN" in gesto_corrente:
            colore = (0, 255, 0) 
            testo_hud = f"Gesto: {gesto_corrente}"
        else:
            colore = (200, 200, 200) 
            testo_hud = "STATUS: IDLE / Waiting"

        cv2.putText(image, testo_hud, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colore, 2, cv2.LINE_AA)

        cv2.imshow('Sensor 3D', image)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        sensore_3d_node()
    except rospy.ROSInterruptException:
        pass
    