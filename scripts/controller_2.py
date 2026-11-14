#!/usr/bin/env python3
"""
OpenManipulator-X — Controllore Pick-and-Place v3 + Emergenza Pugno
ROS 1 Noetic + Gazebo

NOVITÀ rispetto alla versione precedente:
  • Subscriber a /gesto_emergenza (String "STOP") → congela il robot
    nella posizione istantanea in cui si trova
  • Thread keyboard nel terminale del controller → INVIO per riprendere
  • Subscriber a /riprendi_robot (String "VAI") → ripresa alternativa
    dal terminale del sensor_3D
  • _traj() controlla il flag di emergenza ad ogni tick (50 Hz):
    quando scatta, smette di avanzare nella traiettoria e rilancia
    continuamente l'ultimo q inviato finché non arriva il "VAI"
"""

import threading
import rospy
import numpy as np
import math
from typing import List, Optional, Tuple
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

try:
    from gazebo_ros_link_attacher.srv import Attach, AttachRequest
    _ATTACHER_DISPONIBILE = True
except ImportError:
    _ATTACHER_DISPONIBILE = False


# ══════════════════════════════════════════════════════════════════════
#  GEOMETRIA
# ══════════════════════════════════════════════════════════════════════
L1 = 0.077; L2 = 0.130; L3 = 0.124; L4 = 0.126
HOME_HEIGHT = L1 + L2
JOINT_LIMITS: List[Tuple[float, float]] = [
    (-2.8274,  2.8274),
    (-1.7952,  1.7952),
    (-1.8849,  1.7279),
    (-2.0944,  2.0944),
]
HOME_JOINTS  = [0.0, 0.0, 0.0, 0.0]
FK_TOLERANCE = 0.005


# ══════════════════════════════════════════════════════════════════════
#  PARAMETRI
# ══════════════════════════════════════════════════════════════════════
Z_PICK_OFFSET        = 0.0
ROBOT_NOME_GAZEBO    = "open_manipulator"
ROBOT_LINK_GRIPPER   = "gripper_link"
RAGGIO_CERCA_OGGETTO = 0.25

MODELLI_ESCLUSI = [
    "ground_plane", "sun", "robot_pedestal", "industrial_workbench",
    "collection_bin", "table_bin", "overhead_camera",
    "operator_position_marker", "operator_right_industrial_rack",
    "punto_rosso_laser", "target_marker", "ik_target",
    "open_manipulator", ROBOT_NOME_GAZEBO,
]

BIN_JOINTS = [float(math.atan2(-0.25, 0.19)), -0.05, 0.0, 0.0]

GRIPPER_APERTO = 0.014
GRIPPER_CHIUSO = -0.003

T_QUOTA_SICURA  = 3.0
T_HOVER         = 3.0
T_DISCESA_PRESA = 2.0
T_GRIP          = 1.5
T_PAUSA_GRIP    = 1.0
T_LIFT          = 1.5
T_TRANSITO_BIN  = 3.0
T_DISCESA_BIN   = 2.0
T_RISALITA_BIN  = 2.0
T_HOME          = 2.0
COOLDOWN_S      = 2.0

RATE_HZ = 50


#  CINEMATICA

def forward_kinematics(j):
    j1,j2,j3,j4 = j; phi = j2+j3+j4
    r = L2*math.sin(j2) + L3*math.cos(j2+j3) + L4*math.cos(phi)
    z = L1 + L2*math.cos(j2) - L3*math.sin(j2+j3) - L4*math.sin(phi)
    return r*math.cos(j1), r*math.sin(j1), z

def inverse_kinematics(x, y, z, phi=0.0, seconda=False):
    j1 = math.atan2(y,x); r = math.sqrt(x**2+y**2)
    R = r - L4*math.cos(phi); Z = z + L4*math.sin(phi) - L1
    d2 = R**2 + Z**2
    if d2 > (L2+L3)**2+1e-5 or d2 < (L2-L3)**2-1e-5: return None
    s3 = float(np.clip((L2**2+L3**2-d2)/(2*L2*L3), -1, 1))
    j3b = math.asin(s3); j3 = (math.pi-j3b) if seconda else j3b
    A = L2-L3*math.sin(j3); B = L3*math.cos(j3); D2 = A**2+B**2
    if D2 < 1e-10: return None
    sj2 = (R*A-Z*B)/D2; cj2 = (R*B+Z*A)/D2
    if abs(sj2) > 1+1e-4 or abs(cj2) > 1+1e-4: return None
    j2 = math.atan2(sj2,cj2); j4 = phi-j2-j3; jj = [j1,j2,j3,j4]
    for jv,(lo,hi) in zip(jj,JOINT_LIMITS):
        if not (lo-1e-4 <= jv <= hi+1e-4): return None
    fx,fy,fz = forward_kinematics(jj)
    if math.sqrt((fx-x)**2+(fy-y)**2+(fz-z)**2) > FK_TOLERANCE: return None
    return jj

def trova_phi_multi(x, y, z_list, step_deg=5.0):
    distanza_radiale = math.sqrt(x**2+y**2)
    phi_target = 0.0 if distanza_radiale > 0.26 else -math.pi/2.0
    for sec in [False]:
        sols=[]; ok=True
        for z in z_list:
            s = inverse_kinematics(x,y,z,float(phi_target),sec)
            if s is None: ok=False; break
            sols.append(s)
        if ok: return float(phi_target), sols
    step = math.radians(step_deg)
    phi_arr = sorted(np.arange(-math.pi/2,math.pi/2+1e-9,step).tolist(),
                     key=lambda p:(p>0,abs(p+math.pi/2)))
    for sec in [False, True]:
        for phi in phi_arr:
            sols=[]; ok=True
            for z in z_list:
                s = inverse_kinematics(x,y,z,float(phi),sec)
                if s is None: ok=False; break
                sols.append(s)
            if ok: return float(phi), sols
    return None, None

def pianifica_approccio(x, y, z_pick):
    phi_pick, sols_pick = trova_phi_multi(x, y, [z_pick])
    if phi_pick is None: return None, None, None, None
    q_pick = sols_pick[0]
    z_hover = z_pick + 0.05
    phi_hover, sols_hover = trova_phi_multi(x, y, [z_hover])
    z_safe = z_pick + 0.07
    phi_safe, sols_safe = trova_phi_multi(x, y, [z_safe])
    if sols_safe and sols_hover:
        rospy.loginfo(f"  [IK] 3-fasi ✓ Safe={z_safe:.2f} Hover={z_hover:.2f}")
        return z_safe, z_hover, phi_pick, [sols_safe[0], sols_hover[0], q_pick]
    if sols_hover:
        rospy.logwarn(f"  [IK] 2-fasi ⚠ Hover={z_hover:.2f}")
        return None, z_hover, phi_pick, [sols_hover[0], q_pick]
    rospy.logwarn(f"  [IK] Diretto ⚠⚠")
    return None, None, phi_pick, [q_pick]

def cubic_trajectory(q0, qf, t, T):
    tau = max(0.,min(1.,t/T)); s = 3*tau**2-2*tau**3
    return [q0[i]+s*(qf[i]-q0[i]) for i in range(len(q0))]


# ══════════════════════════════════════════════════════════════════════
#  NODO PICK and PLACE
# ══════════════════════════════════════════════════════════════════════
class PickPlaceController:

    def __init__(self):
        self.gripper_target_corrente = GRIPPER_APERTO
        rospy.init_node("omx_pick_place", anonymous=True)

        self._stato  = "IDLE"
        self._lock   = threading.Lock()
        self.q_now   = list(HOME_JOINTS)
        self._oggetto_agganciato = None

        # ── Flag di emergenza ────────────────────────────────────────
        self._emergenza       = False
        self._emergenza_lock  = threading.Lock()
        self._riprendi_event  = threading.Event()

        ns = self._rileva_namespace()
        rospy.loginfo(f"[INIT] Namespace: '{ns}'")

        self._pub_j = [
            rospy.Publisher(f"{ns}joint{i+1}_position/command", Float64, queue_size=1)
            for i in range(4)
        ]
        self._pub_g  = rospy.Publisher(f"{ns}gripper_position/command",     Float64, queue_size=1)
        self._pub_gs = rospy.Publisher(f"{ns}gripper_sub_position/command", Float64, queue_size=1)
        self._pub_stato = rospy.Publisher("/stato_robot", String, queue_size=10, latch=True)
        self._pub_status_vision = rospy.Publisher("/vision/target_status", String, queue_size=1)

        self._usa_attacher = False
        if _ATTACHER_DISPONIBILE:
            try:
                rospy.wait_for_service("/link_attacher_node/attach", timeout=3.0)
                self._attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
                self._detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
                self._usa_attacher = True
                rospy.loginfo("[INIT] link_attacher: ATTIVO ✓")
            except Exception:
                rospy.logwarn("[INIT] link_attacher non disponibile → attrito")
        else:
            rospy.logwarn("[INIT] gazebo_ros_link_attacher assente → attrito")

        rospy.sleep(1.5)
        self._invia_giunti(HOME_JOINTS)
        self._cmd_pinza(GRIPPER_APERTO)
        self._pub_stato.publish(String(data="HOME"))

        rospy.Subscriber("/target_presa_robot",  Point,  self._cb_target)

        # Subscriber emergenza e ripresa 
        rospy.Subscriber("/gesto_emergenza", String, self._cb_emergenza)
        rospy.Subscriber("/riprendi_robot",  String, self._cb_riprendi)

        rospy.loginfo("[INIT] Attesa servizio set_model_state...")
        rospy.wait_for_service("/gazebo/set_model_state")
        self._set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # ── Thread tastiera (INVIO = riprendi, nel terminale del controller) ──
        t_kbd = threading.Thread(target=self._thread_tastiera, daemon=True)
        t_kbd.start()

        fx,fy,fz = forward_kinematics(HOME_JOINTS)
        rospy.loginfo("╔══════════════════════════════════════════════════╗")
        rospy.loginfo("║  OMX Controller + Gesto Emergenza                ║")
        rospy.loginfo(f"║  HOME FK: ({fx:.3f}, {fy:.3f}, {fz:.3f}) m        ║")
        rospy.loginfo("║  PUGNO  → robot si congela istantaneamente       ║")
        rospy.loginfo("║  INVIO  → riprende il movimento (da qui o da     ║")
        rospy.loginfo("║           sensor_3D)                             ║")
        rospy.loginfo("╚══════════════════════════════════════════════════╝\n")

    # ─────────────────────────────────────────────────────────────────
    #  Thread tastiera — INVIO per sbloccare il robot
    # ─────────────────────────────────────────────────────────────────
    def _thread_tastiera(self):
        rospy.loginfo("[TASTIERA] Premi INVIO per riprendere il robot dopo un PUGNO.")
        while not rospy.is_shutdown():
            try:
                input() 
                self._cancella_emergenza("INVIO da terminale controller")
            except EOFError:
                break
            except KeyboardInterrupt:
                break

    # ─────────────────────────────────────────────────────────────────
    #  Callback gesto_emergenza  →  "STOP"
    # ─────────────────────────────────────────────────────────────────
    def _cb_emergenza(self, msg: String):
        if msg.data == "STOP":
            with self._emergenza_lock:
                if not self._emergenza:               
                    self._emergenza = True
                    self._riprendi_event.clear()      
                    self._pub_stato.publish(String(data="EMERGENZA"))
                    rospy.logwarn(
                        "\n" + "═"*52 +
                        "\n  🛑  EMERGENZA: PUGNO rilevato — robot CONGELATO"
                        "\n  Premi INVIO (qui o su sensor_3D) per riprendere"
                        "\n" + "═"*52)

    # ─────────────────────────────────────────────────────────────────
    #  Callback riprendi_robot  →  "VAI"  (da sensor_3D)
    # ─────────────────────────────────────────────────────────────────
    def _cb_riprendi(self, msg: String):
        if msg.data == "VAI":
            self._cancella_emergenza("messaggio /riprendi_robot")

    # ─────────────────────────────────────────────────────────────────
    #  Cancella emergenza (usato sia da tastiera che da topic)
    # ─────────────────────────────────────────────────────────────────
    def _cancella_emergenza(self, origine: str):
        with self._emergenza_lock:
            if self._emergenza:
                self._emergenza = False
                self._riprendi_event.set()    # sblocca il wait in _traj()
                with self._lock:
                    stato = self._stato
                self._pub_stato.publish(String(data=stato))
                rospy.loginfo(
                    f"\n  ✅  RIPRESA da: {origine} — robot in movimento\n")
            else:
                rospy.loginfo("[TASTIERA] Robot già in movimento, nessuna emergenza attiva.")

    # ─────────────────────────────────────────────────────────────────
    #  Namespace
    # ─────────────────────────────────────────────────────────────────
    def _rileva_namespace(self) -> str:
        rospy.sleep(0.5)
        try:
            for t,_ in rospy.get_published_topics():
                if "joint1_position/command" in t:
                    return t.replace("joint1_position/command","")
        except Exception: pass
        return "/"

    # ─────────────────────────────────────────────────────────────────
    #  Comandi hardware
    # ─────────────────────────────────────────────────────────────────
    def _invia_giunti(self, q: List[float]):
        for p,v in zip(self._pub_j, q): p.publish(Float64(v))
        self.q_now = list(q)

    def _cmd_pinza(self, val: float):
        self._pub_g.publish(Float64(val))
        self._pub_gs.publish(Float64(val))
        self.gripper_target_corrente = val

    def _apri_pinza(self):
        rospy.loginfo("    [PINZA] ← Apertura...")
        rate = rospy.Rate(10); t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec()-t0 < T_GRIP:
            self._cmd_pinza(GRIPPER_APERTO); rate.sleep()
        rospy.loginfo("    [PINZA] ← Aperta ✓")

    def _chiudi_pinza(self):
        rospy.loginfo("    [PINZA] → Chiusura...")
        rate = rospy.Rate(10); t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec()-t0 < T_GRIP:
            self._cmd_pinza(GRIPPER_CHIUSO); rate.sleep()
        rospy.loginfo("    [PINZA] → Chiusa ✓")

    # ─────────────────────────────────────────────────────────────────
    #  _traj — traiettoria cubica 
    # ─────────────────────────────────────────────────────────────────
    def _traj(self, q0: List[float], qf: List[float], T: float, label: str = ""):
        """
        Esegue la traiettoria cubica da q0 a qf in T secondi.

        Ad ogni tick (50 Hz) controlla self._emergenza:
          • Se False  → avanza normalmente
          • Se True   → congela: smette di avanzare il tempo di
                        traiettoria e rilancia continuamente l'ultimo
                        q inviato finché _riprendi_event non viene
                        settato (INVIO o /riprendi_robot).
                        Poi riprende DA DOVE ERA RIMASTO (t_congelato).

        In questo modo il robot non fa scatti né ricomincia da capo.
        """
        if label:
            rospy.loginfo(f"    ► {label}  ({T:.1f}s)")

        rate          = rospy.Rate(RATE_HZ)
        t_ini         = rospy.Time.now().to_sec()
        t_congelato   = 0.0     # tempo di traiettoria accumulato prima del freeze
        in_emergenza  = False

        while not rospy.is_shutdown():

            # ── Controlla flag emergenza ──────────────────────────────
            with self._emergenza_lock:
                emergenza_ora = self._emergenza

            if emergenza_ora:
                if not in_emergenza:
                    # Prima volta che entriamo in emergenza in questa traj:
                    # salva il tempo corrente e manda il q attuale
                    t_congelato = rospy.Time.now().to_sec() - t_ini
                    t_congelato = min(t_congelato, T)   # non oltre la fine
                    q_freeze = cubic_trajectory(q0, qf, t_congelato, T)
                    self._invia_giunti(q_freeze)
                    in_emergenza = True

                # Robot congelato: continua a mandare la stessa posizione
                # a 50 Hz per mantenere i motori in coppia
                self._invia_giunti(self.q_now)

                # Aspetta fino a 0.1s per il segnale di ripresa
                # (non blocca il loop: controlla ogni 100ms)
                self._riprendi_event.wait(timeout=0.1)

                if not self._emergenza:
                    # Emergenza cancellata → riprendi dal punto salvato
                    in_emergenza = False
                    # Resetta t_ini in modo che la traiettoria continui
                    # dal t_congelato in avanti
                    t_ini = rospy.Time.now().to_sec() - t_congelato
                    rospy.loginfo("    [TRAJ] Ripresa dalla posizione di freeze ✓")

                rate.sleep()
                continue

            # ── Avanzamento normale ───────────────────────────────────
            dt = rospy.Time.now().to_sec() - t_ini
            self._invia_giunti(cubic_trajectory(q0, qf, dt, T))

            if dt >= T:
                break

            rate.sleep()

        self._invia_giunti(qf)
        rospy.sleep(0.2)

    # ─────────────────────────────────────────────────────────────────
    #  Link attacher
    # ─────────────────────────────────────────────────────────────────
    def _cerca_oggetto(self, x, y, z) -> Optional[str]:
        try:
            from gazebo_msgs.srv import GetWorldProperties
            gw = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
            gs = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            candidati = []
            for nome in gw().model_names:
                if any(e in nome for e in MODELLI_ESCLUSI): continue
                try:
                    pos  = gs(nome,"world").pose.position
                    dist = math.sqrt((pos.x-x)**2+(pos.y-y)**2)
                    if dist < RAGGIO_CERCA_OGGETTO: candidati.append((dist,nome))
                except Exception: pass
            if candidati:
                candidati.sort(); return candidati[0][1]
        except Exception as e:
            rospy.logwarn(f"    [CERCA] {e}")
        return None

    def _aggancia(self, x_pick, y_pick, z_pick):
        if not self._usa_attacher:
            rospy.sleep(T_PAUSA_GRIP); return
        fx,fy,fz = forward_kinematics(self.q_now)
        nome = self._cerca_oggetto(fx,fy,fz)
        if nome is None:
            rospy.logwarn("    [GRIP] Nessun oggetto trovato!")
            rospy.sleep(T_PAUSA_GRIP); return
        try:
            gs = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            stato = gs(nome,"world")
            ms = ModelState(); ms.model_name=nome
            ms.pose=stato.pose; ms.twist=Twist()
            ms.reference_frame="world"
            self._set_model_state_srv.call(ms)
            req = AttachRequest()
            req.model_name_1=ROBOT_NOME_GAZEBO; req.link_name_1=ROBOT_LINK_GRIPPER
            req.model_name_2=nome;              req.link_name_2="link"
            resp = self._attach_srv.call(req)
            if resp.ok:
                self._oggetto_agganciato=(nome,"link")
                rospy.loginfo(f"    [GRIP] ✓ Agganciato: '{nome}'")
            else: rospy.logwarn(f"    [GRIP] attach ok=False per '{nome}'")
        except Exception as e: rospy.logwarn(f"    [GRIP] {e}")
        rospy.sleep(T_PAUSA_GRIP)

    def _sgancia(self):
        if not self._usa_attacher or self._oggetto_agganciato is None: return
        nome,link = self._oggetto_agganciato
        try:
            req = AttachRequest()
            req.model_name_1=ROBOT_NOME_GAZEBO; req.link_name_1=ROBOT_LINK_GRIPPER
            req.model_name_2=nome;              req.link_name_2=link
            self._detach_srv.call(req)
            rospy.loginfo(f"    [GRIP] ✓ Sganciato: '{nome}'")
        except Exception as e: rospy.logwarn(f"    [GRIP] {e}")
        finally: self._oggetto_agganciato = None

    # ─────────────────────────────────────────────────────────────────
    #  Callback target
    # ─────────────────────────────────────────────────────────────────
    def _cb_target(self, msg: Point):
        with self._lock:
            if self._stato != "IDLE":
                rospy.logwarn("[CB] Robot occupato — target ignorato"); return
            self._stato = "BUSY"
        try:
            self._sequenza(msg.x, msg.y, msg.z)
        except Exception as e:
            rospy.logerr(f"[ERRORE] {e}")
            try: self._sgancia()
            except Exception: pass
        finally:
            rospy.loginfo("\n  ── Fine Ciclo: Ripristino ────────────────")
            self._traj(self.q_now, HOME_JOINTS, T_HOME, "RITORNO HOME")
            self._apri_pinza()
            rospy.sleep(COOLDOWN_S)
            self._pub_stato.publish(String(data="HOME"))
            with self._lock: self._stato = "IDLE"

    # ─────────────────────────────────────────────────────────────────
    #  Sequenza pick-and-place
    # ─────────────────────────────────────────────────────────────────
    def _sequenza(self, x: float, y: float, z_pick: float):
        rospy.loginfo(f"\n--- SEQUENZA (Target: {x:.3f}, {y:.3f}, {z_pick:.3f}) ---")

        pianificazione = pianifica_approccio(x, y, z_pick)
        if pianificazione is None or pianificazione[2] is None:
            rospy.logerr("  [ERRORE] Target irraggiungibile!")
            self._pub_status_vision.publish("UNREACHABLE"); return

        self._pub_status_vision.publish("OK")
        z_safe, z_hover, phi_pick, sols_p = pianificazione

        if z_safe is not None and len(sols_p) == 3:
            q_safe, q_hover, q_pick = sols_p
        elif z_hover is not None and len(sols_p) == 2:
            q_safe = None; q_hover, q_pick = sols_p
        else:
            q_safe = None; q_hover = None; q_pick = sols_p[0]

        # FASE 1: Apertura pinza
        rospy.loginfo("-> Fase 1: Apertura pinza")
        self._apri_pinza()
        rospy.sleep(0.2)

        # FASE 2: Avvicinamento in quota
        if q_safe is not None:
            self._traj(self.q_now, q_safe,  T_QUOTA_SICURA, f"Quota sicura Z={z_safe:.3f}")
        if q_hover is not None:
            self._traj(self.q_now, q_hover, T_HOVER,        f"Hover Z={z_hover:.3f}")

        # FASE 3: Discesa sul pezzo
        self._traj(self.q_now, q_pick, T_DISCESA_PRESA, f"Pick Z={z_pick:.3f}")

        # FASE 4: Aggancio
        rospy.loginfo("-> Fase 4: Aggancio")
        rospy.sleep(0.2)
        self._aggancia(x, y, z_pick)

        # FASE 5: Sollevamento
        if q_hover is not None:
            self._traj(self.q_now, q_hover, T_LIFT, "Lift → hover")
        elif q_safe is not None:
            self._traj(self.q_now, q_safe,  T_LIFT, "Lift → safe")

        # FASE 6: Transito verso bin
        self._traj(self.q_now, BIN_JOINTS, T_TRANSITO_BIN, "Transito → bin")

        # FASE 7: Rilascio
        rospy.loginfo("-> Fase 7: Rilascio")
        self._sgancia()
        self._apri_pinza()

        # FASE 8: Disimpegno
        q_dis = list(BIN_JOINTS); q_dis[1] -= 0.1
        self._traj(self.q_now, q_dis, T_RISALITA_BIN, "Disimpegno dal bin")

        rospy.loginfo("-> Sequenza completata ✓")

    # ─────────────────────────────────────────────────────────────────
    #  Run loop
    # ─────────────────────────────────────────────────────────────────
    def run(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            self._invia_giunti(self.q_now)
            self._pub_g.publish(Float64(self.gripper_target_corrente))
            self._pub_gs.publish(Float64(self.gripper_target_corrente))
            rate.sleep()


# ══════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    try:
        PickPlaceController().run()
    except rospy.ROSInterruptException:
        pass