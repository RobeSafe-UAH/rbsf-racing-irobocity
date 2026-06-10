import argparse
import os
import tempfile
from xml.sax.saxutils import escape, quoteattr

import yaml


def _xml_value(value):
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def _xml_text(value):
    return escape(_xml_value(value))


def _xml_attributes(attributes):
    return " ".join(
        f"{name}={quoteattr(_xml_value(value))}"
        for name, value in attributes.items()
    )


def _resolve_config_path(path, config_dir):
    if os.path.isabs(path):
        return path
    return os.path.abspath(os.path.join(config_dir, path))


def _load_vehicle_config(vehicle_config_file):
    with open(vehicle_config_file, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}

    vehicle_config = data.get("vehicle", data)
    if not vehicle_config.get("class_name") and not vehicle_config.get("class"):
        raise RuntimeError(
            f"Vehicle config '{vehicle_config_file}' must define vehicle.class_name"
        )
    return vehicle_config


def _load_world_config(world_config_file):
    if not world_config_file:
        return {}

    with open(world_config_file, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def _resolve_configured_init_pose(world_file, world_config_file, init_pose):
    init_pose = init_pose.strip()
    if init_pose:
        return init_pose

    world_config = _load_world_config(world_config_file)
    worlds = world_config.get("worlds", world_config)
    world_keys = [
        os.path.abspath(world_file),
        os.path.basename(world_file),
        os.path.splitext(os.path.basename(world_file))[0],
    ]

    for world_key in world_keys:
        world_entry = worlds.get(world_key)
        if isinstance(world_entry, dict):
            return world_entry.get("init_pose")
        if isinstance(world_entry, str):
            return world_entry

    default_entry = world_config.get("default", {})
    if isinstance(default_entry, dict):
        return default_entry.get("init_pose")
    if isinstance(default_entry, str):
        return default_entry

    return None


def _build_vehicle_xml(vehicle_config, config_dir, init_pose):
    lines = []
    definition_file = vehicle_config.get("definition_file")
    if definition_file:
        definition_attributes = {
            "file": _resolve_config_path(definition_file, config_dir),
            **vehicle_config.get("definition_attributes", {}),
        }
        lines.append(f"    <include {_xml_attributes(definition_attributes)} />")

    vehicle_attributes = {
        "name": vehicle_config.get("name", "r1"),
        "class": vehicle_config.get("class_name", vehicle_config.get("class")),
    }
    lines.append(f"    <vehicle {_xml_attributes(vehicle_attributes)}>")

    if init_pose:
        lines.append(f"        <init_pose>{_xml_text(init_pose)}</init_pose>")

    for sensor in vehicle_config.get("sensors", []):
        sensor_attributes = {
            "file": _resolve_config_path(sensor["file"], config_dir),
            **sensor.get("attributes", {}),
        }
        lines.append(f"        <include {_xml_attributes(sensor_attributes)} />")

    lines.append("    </vehicle>")
    return "\n".join(lines)


def _ensure_relative_asset_link(generated_root, package_dir, asset_dir_name):
    target = os.path.join(package_dir, asset_dir_name)
    link = os.path.join(generated_root, asset_dir_name)

    if os.path.islink(link) and os.path.realpath(link) != os.path.realpath(target):
        os.unlink(link)

    if not os.path.exists(link) and os.path.exists(target):
        os.symlink(target, link)


def build_configured_world_file(
    world_file,
    vehicle_config_file,
    world_config_file,
    init_pose,
):
    if not vehicle_config_file:
        return world_file

    world_file = os.path.abspath(world_file)
    vehicle_config_file = os.path.abspath(vehicle_config_file)
    world_config_file = os.path.abspath(world_config_file) if world_config_file else ""
    world_dir = os.path.dirname(world_file)
    package_dir = os.path.dirname(world_dir)
    config_dir = os.path.dirname(vehicle_config_file)

    vehicle_config = _load_vehicle_config(vehicle_config_file)
    configured_init_pose = _resolve_configured_init_pose(
        world_file,
        world_config_file,
        init_pose,
    )
    vehicle_xml = _build_vehicle_xml(
        vehicle_config,
        config_dir,
        configured_init_pose,
    )

    with open(world_file, "r", encoding="utf-8") as source_world:
        world_xml = source_world.read()

    closing_tag = "</mvsim_world>"
    closing_index = world_xml.rfind(closing_tag)
    if closing_index < 0:
        raise RuntimeError(f"World file '{world_file}' is missing {closing_tag}")

    configured_world_xml = (
        world_xml[:closing_index].rstrip()
        + "\n"
        + vehicle_xml
        + "\n"
        + world_xml[closing_index:]
    )

    generated_world_root = os.path.join(tempfile.gettempdir(), "rbsf_mvsim_worlds")
    generated_world_dir = os.path.join(generated_world_root, "maps")
    os.makedirs(generated_world_dir, exist_ok=True)
    _ensure_relative_asset_link(generated_world_root, package_dir, "models")
    _ensure_relative_asset_link(generated_world_root, package_dir, "definitions")

    fd, generated_world_file = tempfile.mkstemp(
        prefix="generated_",
        suffix=".world.xml",
        dir=generated_world_dir,
    )

    with os.fdopen(fd, "w", encoding="utf-8") as generated_world:
        generated_world.write(configured_world_xml)

    return generated_world_file


def main():
    parser = argparse.ArgumentParser(
        description="Generate an MVSim world file with a configured vehicle."
    )
    parser.add_argument("--world-file", required=True)
    parser.add_argument("--vehicle-config-file", required=True)
    parser.add_argument("--world-config-file", default="")
    parser.add_argument("--init-pose", default="")
    args = parser.parse_args()

    print(
        build_configured_world_file(
            args.world_file,
            args.vehicle_config_file,
            args.world_config_file,
            args.init_pose,
        )
    )


if __name__ == "__main__":
    main()
