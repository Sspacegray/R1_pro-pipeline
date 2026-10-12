from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

try:
    import PyKDL as kdl
except ImportError:  # pragma: no cover - runtime status reports this
    kdl = None

from .constants import ARM_BASE_LINK, ARM_TIP_LINK, DEFAULT_BASE_FRAME

TORSO_IK_LIMITS_RAD = {
    1: (math.radians(-50.0), math.radians(-40.0)),
    2: (math.radians(-50.0), math.radians(-30.0)),
    3: (math.radians(-20.0), math.radians(20.0)),
}

ARM_IK_FALLBACK_SEEDS_RAD = {
    "left": [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.4, 0.2, -2.0, 0.0, 0.0, 0.0],
    ],
    "right": [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, -0.4, -0.2, -2.0, 0.0, 0.0, 0.0],
    ],
}


def _parse_urdf(path: str) -> tuple[dict[str, dict], dict[str, dict]]:
    root = ET.parse(path).getroot()
    joints_by_name: dict[str, dict] = {}
    child_to_joint: dict[str, dict] = {}
    for joint in root.findall("joint"):
        name = joint.get("name") or ""
        jtype = joint.get("type") or "fixed"
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        if parent_elem is None or child_elem is None:
            continue
        parent = parent_elem.get("link") or ""
        child = child_elem.get("link") or ""
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        origin = joint.find("origin")
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(value) for value in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(value) for value in origin.get("rpy").split()]
        axis = [1.0, 0.0, 0.0]
        axis_elem = joint.find("axis")
        if axis_elem is not None and axis_elem.get("xyz"):
            axis = [float(value) for value in axis_elem.get("xyz").split()]
        limit = None
        limit_elem = joint.find("limit")
        if limit_elem is not None:
            lower = limit_elem.get("lower")
            upper = limit_elem.get("upper")
            if lower is not None and upper is not None:
                limit = (float(lower), float(upper))
        info = {
            "name": name,
            "type": jtype,
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
            "limit": limit,
        }
        joints_by_name[name] = info
        child_to_joint[child] = info
    return joints_by_name, child_to_joint


def _build_kdl_chain(
    child_to_joint: dict[str, dict], base_link: str, tip_link: str
) -> tuple[Any, list[str], list[float], list[float]]:
    if kdl is None:
        raise RuntimeError("PyKDL is not available")
    sequence: list[dict] = []
    link = tip_link
    while link != base_link:
        joint = child_to_joint.get(link)
        if joint is None:
            raise RuntimeError(f"tip '{tip_link}' is not connected to base '{base_link}'")
        sequence.append(joint)
        link = joint["parent"]
    sequence.reverse()

    chain = kdl.Chain()
    joint_names: list[str] = []
    lower: list[float] = []
    upper: list[float] = []
    for info in sequence:
        frame = kdl.Frame(kdl.Rotation.RPY(*info["rpy"]), kdl.Vector(*info["xyz"]))
        jtype = info["type"]
        if jtype in ("revolute", "continuous"):
            axis = frame.M * kdl.Vector(*info["axis"])
            joint = kdl.Joint(info["name"], frame.p, axis, kdl.Joint.RotAxis)
        elif jtype == "prismatic":
            axis = frame.M * kdl.Vector(*info["axis"])
            joint = kdl.Joint(info["name"], frame.p, axis, kdl.Joint.TransAxis)
        else:
            joint = kdl.Joint(info["name"], kdl.Joint.Fixed)
        chain.addSegment(kdl.Segment(info["child"], joint, frame))
        if jtype in ("revolute", "prismatic"):
            joint_names.append(info["name"])
            if info["limit"] is None:
                lower.append(-math.pi)
                upper.append(math.pi)
            else:
                lower.append(float(info["limit"][0]))
                upper.append(float(info["limit"][1]))
        elif jtype == "continuous":
            joint_names.append(info["name"])
            lower.append(-math.pi)
            upper.append(math.pi)
    return chain, joint_names, lower, upper


def _array(values: list[float]) -> Any:
    arr = kdl.JntArray(len(values))
    for index, value in enumerate(values):
        arr[index] = float(value)
    return arr


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(float(lower), min(float(upper), float(value)))


def _locked_bounds(value: float, lower: float, upper: float) -> tuple[float, float, float]:
    center = _clamp(value, lower, upper)
    span = max(0.0, float(upper) - float(lower))
    eps = min(1e-6, span * 0.5) if span > 0.0 else 0.0
    return max(float(lower), center - eps), min(float(upper), center + eps), center


def _frame_to_payload(frame: Any) -> dict[str, list[float]]:
    roll, pitch, yaw = frame.M.GetRPY()
    qx, qy, qz, qw = frame.M.GetQuaternion()
    return {
        "position": [float(frame.p.x()), float(frame.p.y()), float(frame.p.z())],
        "rpy_rad": [float(roll), float(pitch), float(yaw)],
        "rpy_deg": [math.degrees(float(roll)), math.degrees(float(pitch)), math.degrees(float(yaw))],
        "orientation_xyzw": [float(qx), float(qy), float(qz), float(qw)],
    }


class BaseLinkIkSolver:
    """Arm IK with all public targets expressed in base_link."""

    def __init__(
        self,
        urdf_path: str,
        *,
        base_frame: str = DEFAULT_BASE_FRAME,
        max_iter: int = 200,
        eps: float = 1e-5,
        pos_tolerance_m: float = 0.01,
    ) -> None:
        if kdl is None:
            raise RuntimeError("PyKDL is not available")
        path = Path(urdf_path).expanduser()
        if not path.is_file():
            raise RuntimeError(f"full URDF not found: {path}")
        self.urdf_path = str(path)
        self.base_frame = base_frame
        self.max_iter = int(max_iter)
        self.eps = float(eps)
        self.pos_tolerance_m = float(pos_tolerance_m)
        _joints_by_name, child_to_joint = _parse_urdf(str(path))
        self._kin: dict[str, dict[str, Any]] = {}
        for arm in ("left", "right"):
            arm_base = ARM_BASE_LINK[arm]
            tip = ARM_TIP_LINK[arm]
            arm_chain, arm_names, lower, upper = _build_kdl_chain(
                child_to_joint, arm_base, tip
            )
            torso_chain, torso_names, _, _ = _build_kdl_chain(
                child_to_joint, self.base_frame, arm_base
            )
            full_chain, full_names, full_lower, full_upper = _build_kdl_chain(
                child_to_joint, self.base_frame, tip
            )
            lower_array = _array(lower)
            upper_array = _array(upper)
            fk_arm = kdl.ChainFkSolverPos_recursive(arm_chain)
            vel_solver = kdl.ChainIkSolverVel_pinv(arm_chain)
            ik_solver = kdl.ChainIkSolverPos_NR_JL(
                arm_chain,
                lower_array,
                upper_array,
                fk_arm,
                vel_solver,
                self.max_iter,
                self.eps,
            )
            self._kin[arm] = {
                "arm_base": arm_base,
                "tip": tip,
                "arm_chain": arm_chain,
                "arm_names": arm_names,
                "lower": lower,
                "upper": upper,
                "lower_array": lower_array,
                "upper_array": upper_array,
                "fk_arm": fk_arm,
                "ik_solver": ik_solver,
                "vel_solver": vel_solver,
                "torso_chain": torso_chain,
                "torso_names": torso_names,
                "fk_torso": kdl.ChainFkSolverPos_recursive(torso_chain),
                "full_chain": full_chain,
                "full_names": full_names,
                "full_lower": full_lower,
                "full_upper": full_upper,
                "fk_full": kdl.ChainFkSolverPos_recursive(full_chain),
            }

    def describe(self) -> dict[str, Any]:
        return {
            "base_frame": self.base_frame,
            "urdf_path": self.urdf_path,
            "arms": {
                arm: {
                    "arm_base": data["arm_base"],
                    "tip": data["tip"],
                    "arm_joint_names": list(data["arm_names"]),
                    "torso_joint_names_locked": list(data["torso_names"]),
                    "full_joint_names": list(data["full_names"]),
                    "torso_ik_assist": {
                        "enabled_by_request": True,
                        "variable_torso_indices": [2, 3, 4],
                        "limits_deg": {
                            "2": [-50.0, -40.0],
                            "3": [-50.0, -30.0],
                            "4": [-20.0, 20.0],
                        },
                    },
                }
                for arm, data in self._kin.items()
            },
        }

    def fk_current(
        self, arm: str, torso_positions: list[float], arm_positions: list[float]
    ) -> dict[str, Any] | None:
        data = self._kin[arm]
        values = list(torso_positions[: len(data["torso_names"])]) + list(
            arm_positions[: len(data["arm_names"])]
        )
        if len(values) < len(data["full_names"]):
            return None
        frame = kdl.Frame()
        if data["fk_full"].JntToCart(_array(values), frame) < 0:
            return None
        payload = _frame_to_payload(frame)
        payload["frame_id"] = self.base_frame
        payload["arm"] = arm
        return payload

    def solve(
        self,
        arm: str,
        xyz_base: list[float],
        torso_positions: list[float],
        arm_positions: list[float],
        rotation_rpy_base: list[float] | None = None,
        allow_torso_ik: bool = False,
        prefer_fallback_seed: bool = False,
    ) -> dict[str, Any]:
        result = {"ok": False, "message": "", "positions": None, "pos_error_m": None}
        try:
            data = self._kin[arm]
            n_torso = len(data["torso_names"])
            n_arm = len(data["arm_names"])
            if len(torso_positions) < n_torso:
                result["message"] = "no torso feedback; cannot lock torso for base_link IK"
                return result
            if len(arm_positions) < n_arm:
                result["message"] = "no arm feedback; cannot seed IK"
                return result

            if allow_torso_ik:
                return self._solve_with_torso(
                    arm,
                    xyz_base,
                    torso_positions,
                    arm_positions,
                    rotation_rpy_base,
                )

            torso_q = _array([float(v) for v in torso_positions[:n_torso]])
            base_to_arm_base = kdl.Frame()
            if data["fk_torso"].JntToCart(torso_q, base_to_arm_base) < 0:
                result["message"] = "failed to compute current base_link->arm_base transform"
                return result

            if rotation_rpy_base is None:
                current = self.fk_current(arm, torso_positions, arm_positions)
                if current is None:
                    result["message"] = "failed to compute current gripper orientation"
                    return result
                target_rotation_base = kdl.Rotation.Quaternion(
                    *current["orientation_xyzw"]
                )
            else:
                target_rotation_base = kdl.Rotation.RPY(
                    float(rotation_rpy_base[0]),
                    float(rotation_rpy_base[1]),
                    float(rotation_rpy_base[2]),
                )

            target_base = kdl.Frame(
                target_rotation_base,
                kdl.Vector(float(xyz_base[0]), float(xyz_base[1]), float(xyz_base[2])),
            )
            target_arm_base = base_to_arm_base.Inverse() * target_base
            seed_candidates = self._arm_seed_candidates(
                arm,
                arm_positions,
                data,
                prefer_fallback_seed=bool(prefer_fallback_seed),
            )
            best: dict[str, Any] | None = None
            for seed_index, seed_candidate in enumerate(seed_candidates):
                candidate = self._solve_locked_torso_once(
                    data,
                    seed_candidate["values"],
                    target_arm_base,
                    target_base,
                    base_to_arm_base,
                )
                candidate["seed_index"] = seed_index
                candidate["seed_label"] = seed_candidate["label"]
                if best is None or float(candidate["pos_error_m"]) < float(best["pos_error_m"]):
                    best = candidate
                if float(candidate["pos_error_m"]) <= self.pos_tolerance_m:
                    result["ok"] = True
                    result["positions"] = [
                        float(candidate["solution"][i]) for i in range(n_arm)
                    ]
                    result["achieved_pose"] = _frame_to_payload(candidate["achieved_base"])
                    result["pos_error_m"] = float(candidate["pos_error_m"])
                    seed_label = str(candidate.get("seed_label") or "")
                    suffix = "" if seed_label == "current feedback seed" else f" using {seed_label}"
                    result["message"] = f"IK solved in base_link with torso locked{suffix}"
                    return result

            if best is None:
                result["message"] = "IK failed before producing a candidate"
                return result
            result["pos_error_m"] = float(best["pos_error_m"])
            if float(best["pos_error_m"]) > self.pos_tolerance_m:
                result["message"] = (
                    f"IK did not converge after {len(seed_candidates)} seed(s) "
                    f"(best rc={best['rc']}, seed={best['seed_index']}); position error "
                    f"{best['pos_error_m']:.4f} m exceeds tolerance {self.pos_tolerance_m:.4f} m"
                )
                return result
        except Exception as exc:
            result["message"] = f"IK error: {exc}"
            return result

    def _arm_seed_candidates(
        self,
        arm: str,
        arm_positions: list[float],
        data: dict[str, Any],
        *,
        prefer_fallback_seed: bool = False,
    ) -> list[dict[str, Any]]:
        n_arm = len(data["arm_names"])
        fallback_candidates = [
            (f"fallback seed {index}", [float(value) for value in seed])
            for index, seed in enumerate(ARM_IK_FALLBACK_SEEDS_RAD.get(arm, []))
        ]
        current_candidate = (
            "current feedback seed",
            [float(value) for value in arm_positions[:n_arm]],
        )
        if prefer_fallback_seed:
            raw_candidates = [*fallback_candidates, current_candidate]
        else:
            raw_candidates = [current_candidate, *fallback_candidates]
        candidates: list[dict[str, Any]] = []
        seen: set[tuple[float, ...]] = set()
        for label, raw in raw_candidates:
            if len(raw) < n_arm:
                continue
            values = [
                _clamp(float(raw[index]), data["lower"][index], data["upper"][index])
                for index in range(n_arm)
            ]
            key = tuple(round(value, 6) for value in values)
            if key in seen:
                continue
            seen.add(key)
            candidates.append({"label": label, "values": values})
        return candidates

    def _solve_locked_torso_once(
        self,
        data: dict[str, Any],
        seed_values: list[float],
        target_arm_base: Any,
        target_base: Any,
        base_to_arm_base: Any,
    ) -> dict[str, Any]:
        n_arm = len(data["arm_names"])
        seed = _array(seed_values)
        solution = kdl.JntArray(n_arm)
        rc = data["ik_solver"].CartToJnt(seed, target_arm_base, solution)
        for index in range(n_arm):
            solution[index] = _clamp(solution[index], data["lower"][index], data["upper"][index])

        achieved_base = self._achieved_base(data, solution, base_to_arm_base)
        pos_error = (achieved_base.p - target_base.p).Norm()
        if pos_error > self.pos_tolerance_m:
            lma_solution = kdl.JntArray(n_arm)
            lma = kdl.ChainIkSolverPos_LMA(data["arm_chain"])
            lma_rc = lma.CartToJnt(seed, target_arm_base, lma_solution)
            for index in range(n_arm):
                lma_solution[index] = _clamp(
                    lma_solution[index],
                    data["lower"][index],
                    data["upper"][index],
                )
            try:
                lma_achieved_base = self._achieved_base(data, lma_solution, base_to_arm_base)
            except RuntimeError:
                lma_achieved_base = None
            if lma_achieved_base is not None:
                lma_pos_error = (lma_achieved_base.p - target_base.p).Norm()
                if lma_pos_error < pos_error:
                    solution = lma_solution
                    rc = lma_rc
                    achieved_base = lma_achieved_base
                    pos_error = lma_pos_error
        return {
            "solution": solution,
            "achieved_base": achieved_base,
            "pos_error_m": float(pos_error),
            "rc": int(rc),
        }

    def _achieved_base(self, data: dict[str, Any], solution: Any, base_to_arm_base: Any) -> Any:
        achieved_arm = kdl.Frame()
        if data["fk_arm"].JntToCart(solution, achieved_arm) < 0:
            raise RuntimeError("FK verification failed for IK solution")
        return base_to_arm_base * achieved_arm

    def _solve_with_torso(
        self,
        arm: str,
        xyz_base: list[float],
        torso_positions: list[float],
        arm_positions: list[float],
        rotation_rpy_base: list[float] | None,
    ) -> dict[str, Any]:
        result = {"ok": False, "message": "", "positions": None, "pos_error_m": None}
        data = self._kin[arm]
        n_torso = len(data["torso_names"])
        n_arm = len(data["arm_names"])
        n_full = len(data["full_names"])
        if n_full != n_torso + n_arm:
            result["message"] = "full IK chain layout is not torso joints followed by arm joints"
            return result

        if rotation_rpy_base is None:
            current = self.fk_current(arm, torso_positions, arm_positions)
            if current is None:
                result["message"] = "failed to compute current gripper orientation"
                return result
            target_rotation_base = kdl.Rotation.Quaternion(*current["orientation_xyzw"])
        else:
            target_rotation_base = kdl.Rotation.RPY(
                float(rotation_rpy_base[0]),
                float(rotation_rpy_base[1]),
                float(rotation_rpy_base[2]),
            )

        full_lower = [float(value) for value in data["full_lower"]]
        full_upper = [float(value) for value in data["full_upper"]]
        seed_values = [float(value) for value in torso_positions[:n_torso]] + [
            float(value) for value in arm_positions[:n_arm]
        ]
        torso_indices = []
        for index in range(n_torso):
            if index in TORSO_IK_LIMITS_RAD:
                req_lower, req_upper = TORSO_IK_LIMITS_RAD[index]
                lower = max(full_lower[index], req_lower)
                upper = min(full_upper[index], req_upper)
                if lower > upper:
                    result["message"] = (
                        f"torso joint {index + 1} requested IK limit has no overlap "
                        f"with URDF limit"
                    )
                    return result
                full_lower[index] = lower
                full_upper[index] = upper
                seed_values[index] = _clamp(seed_values[index], lower, upper)
                torso_indices.append(index)
            else:
                lower, upper, center = _locked_bounds(
                    seed_values[index], full_lower[index], full_upper[index]
                )
                full_lower[index] = lower
                full_upper[index] = upper
                seed_values[index] = center

        for index in range(n_torso, n_full):
            seed_values[index] = _clamp(seed_values[index], full_lower[index], full_upper[index])

        target_base = kdl.Frame(
            target_rotation_base,
            kdl.Vector(float(xyz_base[0]), float(xyz_base[1]), float(xyz_base[2])),
        )
        seed = _array(seed_values)
        solution = kdl.JntArray(n_full)
        fk_full = data["fk_full"]
        vel_solver = kdl.ChainIkSolverVel_pinv(data["full_chain"])
        ik_solver = kdl.ChainIkSolverPos_NR_JL(
            data["full_chain"],
            _array(full_lower),
            _array(full_upper),
            fk_full,
            vel_solver,
            self.max_iter,
            self.eps,
        )
        rc = ik_solver.CartToJnt(seed, target_base, solution)
        achieved_base = kdl.Frame()
        if fk_full.JntToCart(solution, achieved_base) < 0:
            result["message"] = "FK verification failed for torso-assisted IK solution"
            return result
        pos_error = (achieved_base.p - target_base.p).Norm()
        if pos_error > self.pos_tolerance_m:
            lma_solution = kdl.JntArray(n_full)
            lma = kdl.ChainIkSolverPos_LMA(data["full_chain"])
            lma_rc = lma.CartToJnt(seed, target_base, lma_solution)
            for index in range(n_full):
                lma_solution[index] = _clamp(lma_solution[index], full_lower[index], full_upper[index])
            lma_achieved_base = kdl.Frame()
            if fk_full.JntToCart(lma_solution, lma_achieved_base) >= 0:
                lma_pos_error = (lma_achieved_base.p - target_base.p).Norm()
                if lma_pos_error < pos_error:
                    solution = lma_solution
                    rc = lma_rc
                    achieved_base = lma_achieved_base
                    pos_error = lma_pos_error
        result["pos_error_m"] = float(pos_error)
        if pos_error > self.pos_tolerance_m:
            result["message"] = (
                f"torso-assisted IK did not converge (rc={rc}); position error "
                f"{pos_error:.4f} m exceeds tolerance {self.pos_tolerance_m:.4f} m"
            )
            return result

        torso_solution = [float(solution[i]) for i in range(n_torso)]
        arm_solution = [float(solution[n_torso + i]) for i in range(n_arm)]
        result["ok"] = True
        result["positions"] = arm_solution
        result["torso_positions"] = torso_solution
        result["torso_indices"] = torso_indices
        result["achieved_pose"] = _frame_to_payload(achieved_base)
        torso_deg = [round(math.degrees(torso_solution[i]), 2) for i in torso_indices]
        result["message"] = (
            "IK solved in base_link with torso joints 2/3/4 enabled; "
            f"torso_deg={torso_deg}"
        )
        return result
