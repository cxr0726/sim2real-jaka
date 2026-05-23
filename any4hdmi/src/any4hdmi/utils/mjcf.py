from __future__ import annotations

from pathlib import Path

import mujoco

try:
    from mjhub import resolve_asset_reference as _resolve_asset_reference
except ImportError:
    from mjhub import resolve_mjcf_reference as _resolve_asset_reference


DEFAULT_BASE_JOINT_NAME = "floating_base_joint"
DEFAULT_MJCF_REPO_ID = "elijahgalahad/g1_xmls"
DEFAULT_MJCF_PATH = "g1-mode_13_15.xml"
DEFAULT_MJCF_REVISION = "main"
MjcfInput = str | Path


def build_hf_mjcf_reference(
    *,
    repo_id: str = DEFAULT_MJCF_REPO_ID,
    path: str = DEFAULT_MJCF_PATH,
    revision: str = DEFAULT_MJCF_REVISION,
) -> str:
    return f"hf://{repo_id}@{revision}/{path}"


def _find_local_mjcf_path(namespace: str, repo_name: str, revision: str, repo_path: str) -> Path | None:
    # 1. Look relative to the workspace root containing this source file
    try:
        source_root = Path(__file__).resolve().parents[4]  # /home/irmv/sim2real
        candidates = [
            source_root / repo_name / repo_path,
            source_root / repo_path,
        ]
    except Exception:
        candidates = []

    # 2. Look relative to CWD and its parent
    cwd = Path.cwd().resolve()
    candidates.extend([
        cwd / repo_name / repo_path,
        cwd.parent / repo_name / repo_path,
        cwd / repo_path,
        cwd.parent / repo_path,
    ])

    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def resolve_mjcf_path(mjcf: MjcfInput, *, dataset_root: str | Path | None = None) -> Path:
    normalized = normalize_mjcf_reference(mjcf, dataset_root=dataset_root)
    if isinstance(normalized, Path):
        return normalized

    if isinstance(normalized, str) and normalized.startswith("hf://"):
        try:
            # Parse hf:// reference
            raw = normalized[len("hf://"):]
            parts = raw.split("/", 2)
            if len(parts) == 3:
                namespace = parts[0]
                repo_and_revision = parts[1]
                repo_path = parts[2]
                repo_name, sep, revision = repo_and_revision.partition("@")
                if not sep:
                    revision = "main"

                local_path = _find_local_mjcf_path(namespace, repo_name, revision, repo_path)
                if local_path is not None:
                    print(f"Resolved {normalized} locally to {local_path}")
                    return local_path
        except Exception:
            pass

    return _resolve_asset_reference(normalized)


def normalize_mjcf_reference(
    mjcf: MjcfInput,
    *,
    dataset_root: str | Path | None = None,
) -> MjcfInput:
    if isinstance(mjcf, Path):
        return mjcf.expanduser().resolve()
    if isinstance(mjcf, str):
        if mjcf.startswith("hf://"):
            return mjcf
        base_dir = Path(dataset_root).expanduser().resolve() if dataset_root else Path.cwd()
        return (base_dir / mjcf).resolve()
    raise TypeError(f"Unsupported mjcf payload type: {type(mjcf)!r}")


def qpos_names_from_model(
    model: mujoco.MjModel, base_joint_name: str = DEFAULT_BASE_JOINT_NAME
) -> list[str]:
    names: list[str] = []
    for joint_id in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        joint_type = model.jnt_type[joint_id]
        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            if joint_name == base_joint_name:
                names.extend(
                    [
                        "root_tx",
                        "root_ty",
                        "root_tz",
                        "root_qw",
                        "root_qx",
                        "root_qy",
                        "root_qz",
                    ]
                )
            else:
                names.extend(
                    [
                        f"{joint_name}_tx",
                        f"{joint_name}_ty",
                        f"{joint_name}_tz",
                        f"{joint_name}_qw",
                        f"{joint_name}_qx",
                        f"{joint_name}_qy",
                        f"{joint_name}_qz",
                    ]
                )
        elif joint_type in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            names.append(str(joint_name))
        elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
            names.extend(
                [f"{joint_name}_qw", f"{joint_name}_qx", f"{joint_name}_qy", f"{joint_name}_qz"]
            )
        else:
            raise ValueError(f"Unsupported joint type {joint_type} for {joint_name}")
    if len(names) != model.nq:
        raise ValueError(f"Expected {model.nq} qpos names, got {len(names)}")
    return names
