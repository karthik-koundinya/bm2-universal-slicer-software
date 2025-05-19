from __future__ import annotations

import os
import csv
import time
import threading
from typing import Dict, Any, List, Optional
import numpy as np
import slicer  
import vtk  
import qt  
from urdf_parser_py.urdf import URDF  

class URDFLoaderLogic:  
    def __init__(self):
        self.joint_mapping: Dict[str, Dict[str, Any]] = {}
        self.link_model_nodes: Dict[str, "vtkMRMLModelNode"] = {}
        self.csv_joint_map: Dict[str, str] = {}
        self._motion_thread: threading.Optional[Thread] = None
        self._running = False
        self._robot = None  

    # ------------------------------------------------------------------
    # Robot link
    # ------------------------------------------------------------------
    def attach_robot(self, robot) -> None: 
        """If set, CSV playback also pushes pulses to the physical robot."""
        self._robot = robot

    # ------------------------------------------------------------------
    # URDF loading and rendering
    # ------------------------------------------------------------------
    def load_and_render(self, urdf_path: str) -> None:
        robot = URDF.from_xml_file(urdf_path)
        self._build_joint_map(robot)
        self._render(robot, os.path.dirname(urdf_path))
        slicer.util.infoDisplay("URDF loaded: " + urdf_path)

    # ------------------------------------------------------------------
    def _build_joint_map(self, robot):
        self.joint_mapping.clear()
        for j in robot.joints:
            self.joint_mapping[j.name] = {
                "type": j.joint_type,
                "axis": j.axis or [0, 0, 0],
                "limit": j.limit,
                "parent": j.parent,
                "child": j.child,
                "transform_node": None,
                "baseline": None,
                "origin": getattr(j, "origin", None),
            }
        self._create_joint_transforms()

    # ------------------------------------------------------------------
    def _render(self, robot, base_dir):
        self.link_model_nodes.clear()
        for link in robot.links:
            if not link.visual or not link.visual.geometry or not link.visual.geometry.filename:
                continue
            mesh = os.path.join(base_dir, link.visual.geometry.filename)
            node = slicer.modules.models.logic().AddModel(mesh)
            self.link_model_nodes[link.name] = node
            # parent transform is set later when joints are processed

    def _create_joint_transforms(self):
        for name, data in self.joint_mapping.items():
            tnode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformNode", f"{name}_T")
            data["transform_node"] = tnode
            baseline = vtk.vtkTransform()
            if (origin := data["origin"]):
                if origin.xyz:
                    baseline.Translate(origin.xyz)
                if origin.rpy:
                    baseline.RotateX(np.degrees(origin.rpy[0]))
                    baseline.RotateY(np.degrees(origin.rpy[1]))
                    baseline.RotateZ(np.degrees(origin.rpy[2]))
            data["baseline"] = baseline
            # attach child model
            child = data["child"]
            if child in self.link_model_nodes:
                self.link_model_nodes[child].SetAndObserveTransformNodeID(tnode.GetID())

        # build hierarchy
        for name, data in self.joint_mapping.items():
            parent_link = data["parent"]
            for other_name, other in self.joint_mapping.items():
                if other["child"] == parent_link:
                    data["transform_node"].SetAndObserveTransformNodeID(other["transform_node"].GetID())
                    break

    # ------------------------------------------------------------------
    # CSV‑driven animation
    # ------------------------------------------------------------------
    def play_csv(self, csv_path: str) -> None:
        with open(csv_path, "r", newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            self.csv_joint_map = {h: h for h in header[1:]}
            frames = [list(map(float, row)) for row in reader if row]

        self.stop()
        self._running = True
        self._motion_thread = threading.Thread(target=self._motion_loop, args=(frames,), daemon=True)
        self._motion_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._motion_thread:
            self._motion_thread.join()
        self._motion_thread = None

    def _motion_loop(self, frames: List[List[float]]):
        t0 = time.time()
        for frame in frames:
            if not self._running:
                break
            timestamp = frame[0]
            now = time.time() - t0
            if now < timestamp:
                time.sleep(timestamp - now)
            self._apply_joint_positions(frame[1:])

    def _apply_joint_positions(self, positions: List[float]) -> None:
        for csv_name, pos in zip(self.csv_joint_map, positions):
            j = self.joint_mapping.get(csv_name)
            if not j:
                continue
            tf = vtk.vtkTransform()
            tf.DeepCopy(j["baseline"])
            if j["type"] == "revolute":
                tf.RotateWXYZ(np.degrees(pos), *j["axis"])
            elif j["type"] == "prismatic":
                tf.Translate(*(np.array(j["axis"]) * pos * 1000))
            j["transform_node"].SetMatrixTransformToParent(tf.GetMatrix())

        # # Optionally push pulses to robot (simple heuristic – assumes CSV order)
        # if self._robot:
        #     axis_vec_mm_deg: AxisVec = [positions[0], positions[1], math.degrees(positions[2])]
        #     self._robot._push_axis_vector(axis_vec_mm_deg, interpolate=False)