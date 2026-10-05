#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import numpy as np
import pinocchio as pin
from pinocchio import SE3
from ament_index_python.packages import get_package_share_directory

from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)
from g1pilot.utils.helpers import (
    clamp, wrap_to_pi,
    mat_to_quat_wxyz, quat_wxyz_to_matrix,
    quat_slerp, yaw_from_R,
)


class G1IKSolver:
    """
    Inverse kinematics solver for Unitree G1-29 arms using Pinocchio,
    with smooth orientation control, damping, and optional collision avoidance.
    """

    def __init__(self,
                 urdf_path=None,
                 mesh_dir=None,
                 world_frame='pelvis',
                 frame_left='left_hand_point_contact',
                 frame_right='right_hand_point_contact',
                 alpha=0.2,
                 max_dq_step=0.05,
                 damping=1e-6,
                 max_iter=60,
                 tol=1e-4,
                 pos_gain=1.0,
                 ori_gain=0.8,
                 adaptive_damping=True,
                 sigma_min_thresh=0.08,
                 lambda_base=1e-6,
                 lambda_max=1e-1,
                 max_ori_step_rad=0.35,
                 goal_filter_alpha=0.25,
                 orientation_mode="full",
                 null_space_gain=0.15,
                 debug=False,
                 enable_collision_avoidance=True,
                 collision_margin=0.03):
        self.alpha = alpha
        self.max_dq_step = max_dq_step
        self.damping = damping
        self.max_iter = max_iter
        self.tol = tol
        self.pos_gain = pos_gain
        self.ori_gain = ori_gain
        self.adaptive_damping = adaptive_damping
        self.sigma_min_thresh = sigma_min_thresh
        self.lambda_base = lambda_base
        self.lambda_max = lambda_max
        self.max_ori_step_rad = max_ori_step_rad
        self.goal_filter_alpha = goal_filter_alpha
        self.orientation_mode = orientation_mode
        # Nullraum-Regularisierung: zieht die redundante (7. DOF) Armhaltung sanft zu
        # einer Ruhe-Pose, OHNE die Hand-Pose zu aendern -> der Arm faellt nicht in
        # den "gefalteten" (zum Koerper geklappten) IK-Zweig. 0 = aus.
        self.null_space_gain = null_space_gain
        self._rest_right = None
        self._rest_left = None
        self.debug = debug
        self.world_frame = world_frame
        self.frame_left = frame_left
        self.frame_right = frame_right

        # Selbstkollisions-Gate (s. _init_collision_gate / arm_command_in_collision)
        self.enable_collision_avoidance = enable_collision_avoidance
        self.collision_margin = float(collision_margin)

        # Load model
        try:
            if urdf_path is None or mesh_dir is None:
                pkg_share = get_package_share_directory('g1pilot')
                urdf_path = urdf_path or os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof_inspire_ftp.urdf')
                mesh_dir = mesh_dir or os.path.join(pkg_share, 'description_files', 'meshes')

            # Load kinematic + collision models
            # Compatible version check
            print("Loading URDF model for IK solver...", flush=True)
            try:
                # Try to load both visual and collision geometries
                self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
                    urdf_path,
                    package_dirs=[mesh_dir],
                    geometry_types=[pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]
                )
            except Exception as e:
                print(f"Failed to load collision model properly: {e}", flush=True)
                print("→ Retrying with collision-only model...", flush=True)
                self.model, self.collision_model = pin.buildModelsFromUrdf(
                    urdf_path,
                    package_dirs=[mesh_dir],
                    geometry_types=[pin.GeometryType.COLLISION]
                )
                self.visual_model = None


            self.data = pin.Data(self.model)

        except Exception as e:
            raise RuntimeError(f"Failed to load URDF: {e}")

        # Selbstkollisions-Gate aufbauen (kuratierte Paare + Convex-Hulls).
        # WICHTIG: buildModelsFromUrdf legt KEINE collisionPairs an -- ohne
        # diesen Schritt waere jede Kollisionspruefung ein No-Op (so war es
        # frueher: die "collision avoidance" hat nie etwas geprueft).
        self._gate_ready = False
        if self.enable_collision_avoidance:
            try:
                self._init_collision_gate()
            except Exception as e:
                print(f"[IK] WARN: Kollisions-Gate nicht verfuegbar: {e}", flush=True)

        # Joint mapping
        self._ros_joint_names = [JOINT_NAMES_ROS[i] for i in range(29)]
        self._name_to_q_index = {}
        self._name_to_v_index = {}
        for j in range(1, self.model.njoints):
            jnt = self.model.joints[j]
            if jnt.nq == 1:
                nm = self.model.names[j]
                if nm in self._ros_joint_names:
                    self._name_to_q_index[nm] = jnt.idx_q
                    self._name_to_v_index[nm] = jnt.idx_v

        # Frame IDs
        self._fid_right = self.model.getFrameId(frame_right)
        self._fid_left  = self.model.getFrameId(frame_left)

        # Maximale Armreichweite (Schulter -> Hand-TCP bei gestrecktem Arm, d.h.
        # q=0). Ziele werden in solve() auf diese Kugel um die Schulter geklemmt:
        # der rviz-Marker laesst sich beliebig weit ziehen, aber ein Ziel JENSEITS
        # der Reichweite darf den Solver nicht in wilde Konfigurationen treiben
        # (Ursache fuer "Arm faehrt hinter den Kopf und kommt nie zurueck").
        self._fid_shoulder = {}
        self._arm_reach = {}
        for side, sh_link, fid in (("right", "right_shoulder_pitch_link", self._fid_right),
                                   ("left",  "left_shoulder_pitch_link",  self._fid_left)):
            try:
                fid_sh = self.model.getFrameId(sh_link)
                # Reichweite NICHT bei q=0 messen (dort ist der Arm wegen des
                # TCP-Offsets nicht gestreckt), sondern ueber Ellbogen/Handgelenk
                # samplen und das Maximum nehmen.
                elb = self._name_to_q_index[f"{side}_elbow_joint"]
                wrp = self._name_to_q_index[f"{side}_wrist_pitch_joint"]
                reach = 0.0
                q0 = pin.neutral(self.model)
                for e in np.linspace(-1.0472, 2.0944, 24):
                    for w in np.linspace(-1.6144, 1.6144, 9):
                        q0[elb] = e
                        q0[wrp] = w
                        pin.forwardKinematics(self.model, self.data, q0)
                        pin.updateFramePlacements(self.model, self.data)
                        d = float(np.linalg.norm(
                            self.data.oMf[fid].translation - self.data.oMf[fid_sh].translation))
                        reach = max(reach, d)
                self._fid_shoulder[side] = fid_sh
                self._arm_reach[side] = reach
            except Exception:
                self._fid_shoulder[side] = None
                self._arm_reach[side] = None
        # Sicherheitsmarge: knapp INNERHALB der Kugel bleiben, sonst haengt der
        # Arm dauerhaft in der gestreckten Singulaeritaet.
        self.reach_margin = 0.97

        # Ueberwachte Punkte je Arm fuer die UMGEBUNGS-Kollision (Hindernisse/
        # Greif-Objekte, siehe sync_environment()/environment_command_in_collision()
        # unten): Ellbogen, Handgelenk, Hand-TCP + Zwischenpunkte auf Unterarm und
        # Hand. Nur Ellbogen + TCP liessen den ~25 cm langen Unterarm unbewacht --
        # geplante Bahnen schnitten dann z.B. durch eine Tischkante. Repraesentative
        # Punkte statt voller Mesh-Kollision (billig genug fuer den Planer).
        self._fid_env_elbow = {}
        self._fid_env_wrist = {}
        for side in ("left", "right"):
            for store, link in ((self._fid_env_elbow, "elbow_link"),
                                (self._fid_env_wrist, "wrist_roll_link")):
                try:
                    store[side] = self.model.getFrameId(f"{side}_{link}")
                except Exception:
                    store[side] = None
        # Arm-Punkte (Ellbogen/Unterarm/Handgelenk) brauchen mehr Abstand als der
        # TCP: der Unterarm hat ~4 cm Radius. Die Hand selbst behaelt
        # collision_margin, damit sie z.B. eine Box auf dem Tisch erreicht.
        self.arm_env_margin = 0.06
        # "Hart" (echte Durchdringung) fuer die Arm-Punkte = Linkradius: die
        # Punkte liegen auf der Unterarm-ACHSE, bei Abstand 0 steckte die
        # Oberflaeche schon ~3 cm im Hindernis. Hand-Punkte: 1 cm (eine auf der
        # Tischplatte aufliegende Hand zaehlt als Kollision, nicht als "Band").
        self.arm_env_hard_margin = 0.03
        self.hand_env_hard_margin = 0.01

        # Umgebungs-Objekte (Hindernisse + Greif-Objekte), gesetzt ueber
        # sync_environment(). Alle Posen MUESSEN bereits im world_frame dieses
        # Solvers stehen (Default 'pelvis') -- der Aufrufer (arm_controller.py)
        # transformiert /scene_markers (Frame 'map') per TF dorthin, BEVOR er
        # sync_environment() aufruft.
        self._env_objects = {}
        # Dieselben Objekte als gestapelte Arrays (pos, R, half, is_grasp) fuer
        # den vektorisierten Check - der Planer ruft ihn tausendfach auf, und
        # eine CAD-Zelle bringt Hunderte Objekte mit. Wird in sync_environment()
        # als EIN Tupel ersetzt (atomar fuer den lesenden Regelkreis-Thread).
        self._env_arrays = None
        # Eigenes pin.Data fuer den Umgebungs-Check (analog zu _gate_data oben):
        # die FK hier darf NICHT self.data (Solver-Zustand, von solve()/anderen
        # Aufrufern im selben Tick genutzt) ueberschreiben.
        self._env_data = pin.Data(self.model)

        # State buffers
        self._goal_right = None
        self._goal_left  = None
        self._prev_q_full = None
        self._prev_q14 = None

    # --------------------------------------------------------
    # Helper functions
    # --------------------------------------------------------

    def _limit_ori_step(self, R_cur, R_des):
        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        norm = np.linalg.norm(aa)
        if norm < 1e-12 or norm <= self.max_ori_step_rad:
            return R_des
        aa_limited = aa * (self.max_ori_step_rad / norm)
        return R_cur @ pin.exp3(aa_limited)

    def _lowpass_goal(self, T_prev, T_new):
        if T_prev is None:
            return T_new
        p = (1 - self.goal_filter_alpha) * T_prev.translation + self.goal_filter_alpha * T_new.translation
        q0 = mat_to_quat_wxyz(T_prev.rotation)
        q1 = mat_to_quat_wxyz(T_new.rotation)
        qf = quat_slerp(q0, q1, self.goal_filter_alpha)
        return SE3(quat_wxyz_to_matrix(qf), p)

    # --------------------------------------------------------
    # Selbstkollisions-Gate
    # --------------------------------------------------------
    #
    # Konzept: KEINE Distanz-Abfragen (computeDistances auf den G1-Meshes
    # kostet ~2 s pro Aufruf -> unbrauchbar), sondern ein BINAERER
    # Kollisionscheck mit security_margin auf kuratierten Paaren und
    # Convex-Hull-Geometrien. Gemessen: ~0.3 ms pro Check -> laeuft in
    # jedem 250-Hz-Tick des arm_controller. Der Controller prueft damit die
    # KOMMANDIERTE Konfiguration, BEVOR sie zum Roboter geht, und haelt die
    # Arme an, statt in den eigenen Koerper zu fahren.

    # Bewegliche Armteile, die den Koerper treffen koennen. shoulder_pitch/
    # shoulder_roll bewegen sich kaum vom Torso weg; shoulder_yaw sitzt
    # konstruktionsbedingt dauerhaft ~2.8 cm am Torso und wuerde mit Marge
    # permanent "kollidieren" -> bewusst NICHT gegen den Torso geprueft.
    _GATE_ARM_KEYS = ("elbow", "wrist", "base_link", "palm")
    _GATE_BODY_GEOMS = (
        "torso_link_0", "head_link_0", "pelvis_contour_link_0", "logo_link_0",
        "left_hip_pitch_link_0", "right_hip_pitch_link_0",
        "left_hip_roll_link_0", "right_hip_roll_link_0",
        "left_hip_yaw_link_0", "right_hip_yaw_link_0",
        "left_knee_link_0", "right_knee_link_0",
    )

    def _init_collision_gate(self):
        cm = self.collision_model
        # 1) Meshes -> Convex-Hulls: schneller UND konservativer (Hull
        #    umschliesst das Mesh). Nicht jede hppfcl/coal-Version kann das ->
        #    per hasattr abgesichert; ohne Hulls funktioniert der Check auch,
        #    nur langsamer.
        for g in cm.geometryObjects:
            geom = g.geometry
            if hasattr(geom, "buildConvexRepresentation"):
                try:
                    geom.buildConvexRepresentation(False)
                    if geom.convex is not None:
                        g.geometry = geom.convex
                except Exception:
                    pass

        # 2) Kuratierte Paare: bewegliche Armteile gegen Koerper + Arm gegen Arm.
        names = {g.name for g in cm.geometryObjects}
        moving = {
            side: [n for n in names
                   if n.startswith(side) and any(k in n for k in self._GATE_ARM_KEYS)
                   # Finger-Geometrien nicht pruefen: die Hand darf greifen;
                   # die Handbasis (base_link/palm) deckt den Koerperschutz ab.
                   and not any(f in n for f in ("index", "middle", "ring",
                                                "little", "thumb"))]
            for side in ("left", "right")
        }
        body = [n for n in self._GATE_BODY_GEOMS if n in names]
        for side in ("left", "right"):
            for a in moving[side]:
                for b in body:
                    cm.addCollisionPair(pin.CollisionPair(
                        cm.getGeometryId(a), cm.getGeometryId(b)))
        for a in moving["left"]:
            for b in moving["right"]:
                cm.addCollisionPair(pin.CollisionPair(
                    cm.getGeometryId(a), cm.getGeometryId(b)))

        # 3) Zwei GeometryData: mit Sicherheitsmarge (Fruehwarnband) und hart
        #    (echte Durchdringung). Eigenes pin.Data, damit der Gate-FK nicht
        #    self.data (Solver-Zustand) ueberschreibt.
        self._gate_data = pin.Data(self.model)
        self._gate_cdata_margin = pin.GeometryData(cm)
        self._gate_cdata_hard = pin.GeometryData(cm)
        try:
            for req in self._gate_cdata_margin.collisionRequests:
                req.security_margin = self.collision_margin
        except Exception:
            # Alte hppfcl ohne security_margin: Marge entfaellt, harter
            # Check bleibt wirksam.
            self._gate_cdata_margin = self._gate_cdata_hard
        self._gate_ready = True
        print(f"[IK] Kollisions-Gate aktiv: {len(cm.collisionPairs)} Paare, "
              f"Marge {self.collision_margin*100:.0f} cm.", flush=True)

    # --------------------------------------------------------
    # Umgebungs-Kollision (Hindernisse + Greif-Objekte)
    # --------------------------------------------------------
    #
    # Bewusst KEINE hppfcl/pinocchio-GeometryModel-Erweiterung (anders als das
    # Selbstkollisions-Gate oben): die genaue Konstruktor-Signatur von
    # pin.GeometryObject/hppfcl-Shapes variiert zwischen Pinocchio-Versionen
    # (siehe schon der hasattr-Abwaegung in _init_collision_gate), und ein
    # falscher Aufruf wuerde erst zur LAUFZEIT im 250-Hz-Regelkreis auffliegen.
    # Stattdessen: einfache, klar nachrechenbare Punkt-zu-orientierter-Box-
    # Distanz (Ellbogen + Hand-TCP je Arm gegen jedes Umgebungs-Objekt) -- exakt
    # dieselbe "ein paar Punkte statt volle Mesh-Kollision"-Faustregel wie beim
    # kartesischen Speedlimit im arm_controller.

    def sync_environment(self, objects) -> None:
        """Ersetzt die bekannten Umgebungs-Objekte durch `objects` (Iterable von
        dicts: name, cls ('obstacle'|'grasp'), half_extents (hx,hy,hz), pos
        (3,), quat (4, w-x-y-z)) -- ALLE Posen bereits in self.world_frame
        (Default 'pelvis'). Wird von arm_controller.py bei jedem /scene_markers-
        Update aufgerufen (nach TF-Transformation map->pelvis)."""
        new_objects = {}
        for o in objects:
            try:
                name = o["name"]
                pos = np.asarray(o["pos"], dtype=float).reshape(3)
                quat = np.asarray(o["quat"], dtype=float).reshape(4)
                half = np.asarray(o["half_extents"], dtype=float).reshape(3)
                R = quat_wxyz_to_matrix(quat)
            except (KeyError, ValueError, TypeError):
                continue
            new_objects[name] = {
                "cls": o.get("cls", "obstacle"),
                "pos": pos, "R": R, "half": half,
            }
        self._env_objects = new_objects
        objs = list(new_objects.values())
        self._env_arrays = None if not objs else (
            np.stack([o["pos"] for o in objs]),
            np.stack([o["R"] for o in objs]),
            np.stack([o["half"] for o in objs]),
            np.array([o["cls"] == "grasp" for o in objs]),
        )

    @staticmethod
    def _point_obb_distance(p: np.ndarray, obj: dict) -> float:
        """Kuerzeste Distanz von Punkt p zur Oberflaeche der orientierten Box
        `obj` (0.0, wenn p INNERHALB liegt). Standardformel: Punkt in
        Box-lokale Achsen drehen, Ueberschuss je Achse ueber die Halbextents
        clippen, Norm des Rests = Abstand."""
        local = obj["R"].T @ (p - obj["pos"])
        excess = np.maximum(np.abs(local) - obj["half"], 0.0)
        return float(np.linalg.norm(excess))

    def environment_command_in_collision(self, current_all, hard=False, data=None) -> bool:
        """True, wenn die 29-DOF-Konfiguration current_all (ROS-Gelenkreihen-
        folge) einen ueberwachten Arm-Punkt (Ellbogen, Unterarm, Handgelenk,
        Hand, Hand-TCP) zu nah an ein Umgebungs-Objekt bringt (hard=True: echte
        Durchdringung -- arm_env_hard_margin bzw. hand_env_hard_margin; sonst
        arm_env_margin bzw. collision_margin).

        ACM (Allowed-Collision, siehe g1pilot/docs/11_arm_manipulation_technik.md (Umgebungs-Kollisionsgate)):
        fuer Greif-Objekte (cls='grasp') werden NUR die Hand-Punkte ausgenommen
        (die Hand darf ans Greifziel heran) -- Ellbogen/Unterarm/Handgelenk
        bleiben gegen JEDES Objekt (auch Greif-Objekte) geprueft, und
        Hindernisse (cls='obstacle') werden fuer ALLE Punkte geprueft.

        `data`: optionaler eigener pin.Data-Scratch-Puffer (Default:
        self._env_data). Aufrufer, die VIELE Checks aus einem ANDEREN Thread
        machen (z.B. arm_planner.py waehrend der Bahnplanung), sollten ihren
        eigenen Puffer uebergeben (siehe make_scratch_buffers()) -- sonst
        wuerden sie sich mit dem 250-Hz-Regelkreis denselben mutable Zustand
        teilen (Pinocchio-Kontrakt: Model ist threadsicher/read-only, Data
        ist mutabler Scratch-Zustand pro Aufrufer)."""
        arrays = self._env_arrays   # EINMAL lesen: sync_environment() ersetzt es atomar
        if arrays is None:
            return False
        data = data if data is not None else self._env_data
        q = pin.neutral(self.model)
        for jid_idx, ros_name in enumerate(self._ros_joint_names):
            if ros_name in self._name_to_q_index:
                q[self._name_to_q_index[ros_name]] = float(current_all[jid_idx])
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)

        # Punkte sammeln: (Position, Marge, gilt_gegen_Greif-Objekte).
        # Arm (Ellbogen, 2x Unterarm): gegen ALLE Objekte, groessere Marge.
        # Hand (Handgelenk, Handgelenk->TCP-Mitte, TCP): collision_margin und nur
        # gegen Hindernisse -- an ein Greif-Objekt darf die Hand heran (ACM).
        arm_margin = self.arm_env_hard_margin if hard else self.arm_env_margin
        hand_margin = self.hand_env_hard_margin if hard else self.collision_margin
        pts, margins, vs_grasp = [], [], []
        for side, fid_hand in (("left", self._fid_left), ("right", self._fid_right)):
            fe, fw = self._fid_env_elbow.get(side), self._fid_env_wrist.get(side)
            p_e = data.oMf[fe].translation if fe is not None else None
            p_w = data.oMf[fw].translation if fw is not None else None
            p_h = data.oMf[fid_hand].translation if fid_hand is not None else None
            arm = [p_e] if p_e is not None else []
            if p_e is not None and p_w is not None:
                arm += [p_e + (p_w - p_e) * t for t in (1.0 / 3.0, 2.0 / 3.0)]
            hand = [p for p in (p_w, p_h) if p is not None]
            if p_w is not None and p_h is not None:
                hand.append(0.5 * (p_w + p_h))
            for p in arm:
                pts.append(p); margins.append(arm_margin); vs_grasp.append(True)
            for p in hand:
                pts.append(p); margins.append(hand_margin); vs_grasp.append(False)
        if not pts:
            return False

        P, R, H, is_grasp = arrays
        pts = np.asarray(pts)                                   # (M,3)
        # Punkte in die Box-Achsen drehen: local[m,n] = R[n]^T (p[m] - P[n])
        local = np.einsum("nji,mnj->mni", R, pts[:, None, :] - P[None, :, :])
        excess = np.maximum(np.abs(local) - H[None, :, :], 0.0)
        dist = np.linalg.norm(excess, axis=2)                   # (M,N)
        hit = dist < np.asarray(margins)[:, None]
        hit &= ~(~np.asarray(vs_grasp)[:, None] & is_grasp[None, :])   # Hand vs Greif-Objekt erlaubt
        return bool(hit.any())

    def make_scratch_buffers(self) -> dict:
        """Eigener, unabhaengiger FK-/Kollisions-Scratch-Zustand (eigenes
        pin.Data + eigene GeometryData mit derselben Margen-Konfiguration wie
        das Selbstkollisions-Gate) -- fuer Aufrufer, die (wie arm_planner.py)
        VIELE Kollisionschecks aus einem ANDEREN Thread machen, ohne die
        laufzeitkritischen Puffer dieses Solvers (self.data/self._gate_data/
        self._env_data, vom 250-Hz-Regelkreis genutzt) anzufassen. Pinocchio-
        Kontrakt: Model/GeometryModel sind read-only und threadsicher teilbar,
        nur Data/GeometryData sind mutabler Scratch-Zustand -- jeder
        nebenlaeufige Aufrufer braucht also SEINE EIGENEN."""
        cdata_margin = cdata_hard = None
        if self._gate_ready:
            cdata_margin = pin.GeometryData(self.collision_model)
            cdata_hard = pin.GeometryData(self.collision_model)
            try:
                for req in cdata_margin.collisionRequests:
                    req.security_margin = self.collision_margin
            except Exception:
                cdata_margin = cdata_hard
        return {
            "data": pin.Data(self.model),
            "env_data": pin.Data(self.model),
            "cdata_margin": cdata_margin,
            "cdata_hard": cdata_hard,
        }

    def arm_command_in_collision(self, current_all, hard=False, data=None, cdata=None) -> bool:
        """True, wenn die 29-DOF-Konfiguration current_all (ROS-Gelenkreihenfolge)
        eine Selbstkollision (hard=True) bzw. eine Annaeherung unter die
        Sicherheitsmarge (hard=False) enthaelt. Bei nicht verfuegbarem Gate
        immer False (Verhalten wie bisher, aber der Controller loggt das).

        `data`/`cdata`: optionale eigene Scratch-Puffer (siehe
        make_scratch_buffers()/environment_command_in_collision() fuer die
        Begruendung -- Nebenlaeufigkeit mit dem 250-Hz-Regelkreis)."""
        if not self._gate_ready:
            return False
        q = pin.neutral(self.model)
        for jid_idx, ros_name in enumerate(self._ros_joint_names):
            if ros_name in self._name_to_q_index:
                q[self._name_to_q_index[ros_name]] = float(current_all[jid_idx])
        data = data if data is not None else self._gate_data
        if cdata is None:
            cdata = self._gate_cdata_hard if hard else self._gate_cdata_margin
        return bool(pin.computeCollisions(
            self.model, data, self.collision_model, cdata, q, True))



    # --------------------------------------------------------
    # Public API
    # --------------------------------------------------------

    def set_current_configuration(self, q_dict: dict):
        """
        Manually set the current joint configuration for the solver.
        This is useful to reset the internal state (e.g., after homing).

        Parameters
        ----------
        q_dict : dict
            Dictionary with optional keys 'left' and/or 'right', each
            containing a 7-element numpy array of joint angles in radians.
        """
        q_full = pin.neutral(self.model)

        if 'left' in q_dict:
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                q_full[self._name_to_q_index[self._ros_joint_names[arm_i]]] = q_dict['left'][i]

        if 'right' in q_dict:
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                q_full[self._name_to_q_index[self._ros_joint_names[arm_i]]] = q_dict['right'][i]

        self._prev_q_full = q_full.copy()
        self._prev_q14 = np.concatenate([
            q_dict.get('left', np.zeros(7)),
            q_dict.get('right', np.zeros(7))
        ])


    def set_rest_posture(self, side, q7):
        """Ruhe-Pose (7 DOF) fuer die Nullraum-Regularisierung setzen. Der redundante
        Freiheitsgrad wird sanft hierhin gezogen, ohne die Hand-Pose zu beeinflussen."""
        arr = np.array(q7, dtype=float).copy()
        if side == "right":
            self._rest_right = arr
        elif side == "left":
            self._rest_left = arr

    def set_goal(self, side, T_goal: SE3):
        """Set a new SE3 target for the given side ('left' or 'right')."""
        if side == "right":
            self._goal_right = self._lowpass_goal(self._goal_right, T_goal)
        elif side == "left":
            self._goal_left = self._lowpass_goal(self._goal_left, T_goal)

    def clear_goals(self):
        """Beide IK-Ziele verwerfen (z.B. beim Homing). Wurde vom arm_controller
        schon immer via hasattr aufgerufen, existierte aber nicht -> alte Ziele
        blieben ueber das Homing hinaus aktiv."""
        self._goal_right = None
        self._goal_left = None

    def get_joint_targets(self, current_all: np.ndarray, q_init: np.ndarray = None):
        """
        Compute current joint targets (left/right arms) given current joint positions.
        Returns a dictionary with 'left' and/or 'right' arrays (7 DOF each).
        """
        out = {}
        if self._goal_left is not None:
            q_left = self.solve('left', q_init, current_all)
            if q_left is not None:
                out['left'] = q_left
        if self._goal_right is not None:
            q_right = self.solve('right', q_init, current_all)
            if q_right is not None:
                out['right'] = q_right
        return out

    def _clamp_goal_to_reach(self, side, goal, q, data=None):
        """Ziel-Translation auf die Reichweiten-Kugel um die Schulter klemmen.
        Unerreichbare Ziele (Marker zu weit gezogen) wuerden den DLS-Solver sonst
        durch die gestreckte Singulaeritaet in den gefalteten/ueber-Kopf-Zweig
        druecken. Geklemmt zeigt der Arm stattdessen sauber Richtung Ziel."""
        fid_sh = self._fid_shoulder.get(side)
        reach = self._arm_reach.get(side)
        if fid_sh is None or reach is None:
            return goal
        data = data if data is not None else self.data
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        p_sh = data.oMf[fid_sh].translation
        v = goal.translation - p_sh
        d = float(np.linalg.norm(v))
        r_max = self.reach_margin * reach
        if d <= r_max or d < 1e-9:
            return goal
        return SE3(goal.rotation.copy(), p_sh + v * (r_max / d))

    # --------------------------------------------------------
    # Einmaliges (offline) IK -- fuer eingespielte kartesische Ziele
    # --------------------------------------------------------
    #
    # solve() ist auf EINEN Servo-Schritt ausgelegt (Trust-Region pro Iteration,
    # begrenzter Orientierungsschritt, tiefpassgefiltertes Ziel via set_goal) und
    # arbeitet auf dem GETEILTEN Zustand `self.data` + `self._goal_{left,right}`.
    # Fuer die Live-Pose-Schnittstelle (arm_command / arm_api) brauchen wir das
    # Gegenteil: eine EINMAL auskonvergierte Loesung, berechnet in einem
    # HINTERGRUND-THREAD, ohne das laufende Marker-Servoing zu stoeren.
    # Deshalb ist die getunte Iteration hier data-parametriert (_solve_to_goal)
    # und solve_pose() legt sich eigene Buffer an -- kein gemeinsamer Zustand,
    # keine Race mit dem 250-Hz-Tick, kein Ueberschreiben des Marker-Ziels.

    def _arm_indices(self, side):
        return RIGHT_JOINT_INDICES_LIST if side == 'right' else LEFT_JOINT_INDICES_LIST

    def _q_full_from(self, q_init, current_all):
        """Vollen Konfigurationsvektor aus dem Messvektor aufbauen."""
        q = q_init.copy() if q_init is not None else pin.neutral(self.model)
        for jid_idx, ros_name in enumerate(self._ros_joint_names):
            if ros_name in self._name_to_q_index:
                q[self._name_to_q_index[ros_name]] = float(current_all[jid_idx])
        return q

    def _set_arm_in_q(self, q_full, side, q7):
        for i, arm_i in enumerate(self._arm_indices(side)):
            q_full[self._name_to_q_index[self._ros_joint_names[arm_i]]] = float(q7[i])

    def _frame_placement(self, side, q_full, data):
        fid = self._fid_right if side == 'right' else self._fid_left
        pin.forwardKinematics(self.model, data, q_full)
        pin.updateFramePlacements(self.model, data)
        return data.oMf[fid]

    def solve_pose(self, side: str, T_goal: SE3, current_all: np.ndarray, *,
                   rounds: int = 12, pos_tol: float = 0.005,
                   ori_tol_rad: float = 0.05, clamp_reach: bool = True) -> dict:
        """Kartesische Ziel-Pose EINMAL auskonvergieren (kein Servo-Schritt).

        Ruft die getunte DLS-Iteration mehrfach auf -- ein einzelner Aufruf ist
        durch Trust-Region und `max_ori_step_rad` absichtlich gedeckelt und
        erreicht ein weit entferntes Ziel nicht. Nach jeder Runde wird der
        tatsaechliche Rest-Fehler gemessen und die BESTE Konfiguration behalten
        (die Iteration kann sich in Grenzlagen verschlechtern).

        THREAD-SICHER: eigene pin.Data-Buffer, kein Zugriff auf self.data und
        kein set_goal -- das Marker-Ziel des laufenden Servoings bleibt intakt.

        Returns
        -------
        dict mit 'q' (7 DOF oder None), 'pos_err_m', 'ori_err_deg', 'converged',
        'rounds_used'. `converged=False` heisst: Ziel nicht (genau) erreichbar --
        der Aufrufer entscheidet, ob er die Naeherung trotzdem anfahren will.
        """
        data = pin.Data(self.model)
        q_full = self._q_full_from(None, current_all)
        goal = (self._clamp_goal_to_reach(side, T_goal, q_full, data)
                if clamp_reach else T_goal)

        best_q, best_pos, best_ori, used = None, float("inf"), float("inf"), 0
        for r in range(max(1, int(rounds))):
            q7 = self._solve_to_goal(side, goal, q_full, data)
            if q7 is None:
                break
            self._set_arm_in_q(q_full, side, q7)
            M = self._frame_placement(side, q_full, data)
            pos_err = float(np.linalg.norm(M.translation - T_goal.translation))
            ori_err = float(np.linalg.norm(pin.log3(M.rotation.T @ T_goal.rotation)))
            used = r + 1
            # "Besser" = naeher an der Position; die Orientierung laeuft im
            # Nullraum der Position (siehe _solve_to_goal) und ist zweitrangig.
            if (pos_err, ori_err) < (best_pos, best_ori):
                best_q, best_pos, best_ori = np.array(q7, dtype=float), pos_err, ori_err
            if pos_err <= pos_tol and ori_err <= ori_tol_rad:
                break

        return {
            "q": best_q,
            "pos_err_m": best_pos if best_q is not None else float("inf"),
            "ori_err_deg": math.degrees(best_ori) if best_q is not None else float("inf"),
            "converged": bool(best_q is not None and best_pos <= pos_tol
                              and best_ori <= ori_tol_rad),
            "rounds_used": used,
        }

    def solve(self, side: str, q_init: np.ndarray, current_all: np.ndarray) -> np.ndarray:
        """Compute 7-DOF IK solution for one arm (collision-aware).

        EIN Servo-Schritt gegen das per set_goal gesetzte Ziel (Marker-Pfad,
        250-Hz-Tick). Fuer ein einmalig auskonvergiertes Ziel: solve_pose()."""
        goal = self._goal_right if side == 'right' else self._goal_left
        if goal is None:
            return None
        q = self._q_full_from(q_init, current_all)
        # Unerreichbare Ziele auf die Reichweiten-Kugel klemmen (s.o.).
        goal = self._clamp_goal_to_reach(side, goal, q)
        return self._solve_to_goal(side, goal, q, self.data)

    def _solve_to_goal(self, side: str, goal: SE3, q: np.ndarray, data) -> np.ndarray:
        """Getunte DLS-Iteration auf `goal`, auf den uebergebenen Buffern.
        `q` (voller Konfigurationsvektor) wird in place iteriert; zurueck kommt
        die Arm-Teilmenge (7 DOF)."""
        fid = self._fid_right if side == 'right' else self._fid_left
        arm_ids = self._arm_indices(side)

        # Startfehler merken: falls die Iteration das Ziel VERSCHLECHTERT
        # (Divergenz, z.B. nahe Singulaeritaet), geben wir unten die
        # Ausgangskonfiguration zurueck statt einer wilden Pose.
        q_seed_arm = np.array(
            [q[self._name_to_q_index[self._ros_joint_names[i]]] for i in arm_ids])
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        err0 = np.linalg.norm(pin.log(data.oMf[fid].inverse() * goal).vector)

        # Iterative IK loop
        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, data, q)
            pin.updateFramePlacements(self.model, data)
            M_cur = data.oMf[fid]
            R_des = self._limit_ori_step(M_cur.rotation, goal.rotation)
            T_des = SE3(R_des, goal.translation)

            # Frame Jacobian. WICHTIG: pin.LOCAL, weil der Pose-Fehler unten ein
            # LOCAL-Twist ist (pin.log der Relativtransformation). Frueher stand
            # hier LOCAL_WORLD_ALIGNED -> Jacobian und Fehler waren um die
            # aktuelle Handrotation gegeneinander verdreht -> der Solver schob
            # den Arm systematisch in die falsche Richtung (nach oben/hinter den
            # Kopf statt zum Marker) und blieb dort haengen.
            J6 = pin.computeFrameJacobian(self.model, data, q, fid, pin.LOCAL)
            J_eff = J6[:, [self._name_to_v_index[self._ros_joint_names[i]] for i in arm_ids]]

            # Pose error (LOCAL-Twist, konsistent zur LOCAL-Jacobian)
            err6 = pin.log(M_cur.inverse() * T_des).vector

            if np.linalg.norm(err6) < self.tol:
                break

            # Trust-Region: Fehler pro Iteration begrenzen, damit der DLS-Schritt
            # im linearen Bereich bleibt. Bei weit gezogenem Marker (0.3+ m) wuerde
            # J^+ * err sonst riesige dq liefern, die das Clipping unten richtungs-
            # verfaelscht -> der Arm laeuft schraeg weg statt zum Ziel.
            p_norm = float(np.linalg.norm(err6[:3]))
            if p_norm > 0.10:
                err6[:3] *= 0.10 / p_norm
            err6[:3] *= self.pos_gain
            err6[3:] *= self.ori_gain

            def _damping(J):
                lam = self.lambda_base
                if self.adaptive_damping:
                    svals = np.linalg.svd(J, compute_uv=False)
                    sigma_min = float(np.min(svals)) if len(svals) > 0 else 0.0
                    if sigma_min < self.sigma_min_thresh:
                        frac = clamp((self.sigma_min_thresh - sigma_min) / self.sigma_min_thresh, 0, 1)
                        lam = self.lambda_base + frac * (self.lambda_max - self.lambda_base)
                return lam

            def _dpinv(J):
                lam = _damping(J)
                return J.T @ np.linalg.inv(J @ J.T + lam * np.eye(J.shape[0]))

            # TASK-PRIORITAET statt gemischtem 6D-DLS: Position ist Primaeraufgabe,
            # Orientierung laeuft im Nullraum der Position. An der Reichweiten-
            # grenze (Marker weit gezogen) opferte der gemischte Ansatz sonst
            # die POSITION fuer eine perfekte Orientierung -> Hand blieb weit vor
            # dem Marker stehen / verrenkte sich. So faehrt die Hand immer so nah
            # wie moeglich ans Ziel und nutzt den Rest fuer die Orientierung.
            J_p, J_o = J_eff[:3, :], J_eff[3:, :]
            e_p, e_o = err6[:3], err6[3:]
            Jp_pinv = _dpinv(J_p)
            dq_red = Jp_pinv @ e_p
            N_p = np.eye(len(arm_ids)) - Jp_pinv @ J_p
            J_on = J_o @ N_p
            dq_red = dq_red + _dpinv(J_on) @ (e_o - J_o @ dq_red)

            # Nullraum-Haltungs-Regularisierung: zieht die redundante Armhaltung zur
            # Ruhe-Pose, PROJIZIERT in den Nullraum der Hand-Aufgabe (N = I - J^+ J) ->
            # die Hand-Pose bleibt unveraendert, aber der Arm faellt nicht in den zum
            # Koerper gefalteten Zweig (Ursache fuer das "faehrt in falsche Richtung").
            rest = self._rest_right if side == 'right' else self._rest_left
            if rest is not None and self.null_space_gain > 0.0:
                q_arm = np.array(
                    [q[self._name_to_q_index[self._ros_joint_names[i]]] for i in arm_ids])
                J_pinv = _dpinv(J_eff)
                N = np.eye(len(arm_ids)) - J_pinv @ J_eff
                dq_red += N @ (self.null_space_gain * (rest - q_arm))

            # Integrate step. Schritt RICHTUNGSERHALTEND skalieren (statt per
            # Gelenk zu clippen): per-Gelenk-Clipping verdreht die Schrittrichtung,
            # sobald mehrere Gelenke im Limit sind -> der Arm driftet seitlich weg.
            max_abs = float(np.max(np.abs(dq_red))) if len(dq_red) else 0.0
            if max_abs > self.max_dq_step:
                dq_red = dq_red * (self.max_dq_step / max_abs)
            for i, arm_i in enumerate(arm_ids):
                qi = self._name_to_q_index[self._ros_joint_names[arm_i]]
                q[qi] = np.clip(q[qi] + dq_red[i], *JOINT_LIMITS_RAD[arm_i])

        # Divergenz-Schutz: hat die Iteration den Pose-Fehler VERGROESSERT
        # (Singulaeritaet/Grenzlage), lieber die Ausgangskonfiguration halten,
        # statt eine entgleiste Loesung zu kommandieren.
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        err1 = np.linalg.norm(pin.log(data.oMf[fid].inverse() * goal).vector)
        if err1 > err0 + 1e-6:
            if self.debug:
                print(f"[IK] {side}: divergiert (err {err0:.4f} -> {err1:.4f}), halte Pose.",
                      flush=True)
            return q_seed_arm

        # Return arm subset (7 joints)
        return np.array([float(q[self._name_to_q_index[self._ros_joint_names[i]]]) for i in arm_ids], dtype=float)

