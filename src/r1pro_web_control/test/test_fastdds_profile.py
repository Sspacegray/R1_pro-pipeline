from pathlib import Path
import xml.etree.ElementTree as ET


PROFILE_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "fastdds_workstation_udp.xml"
)
NAMESPACE = {"fastdds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}


def test_default_rmw_endpoints_use_ros_humble_profile_tags() -> None:
    root = ET.parse(PROFILE_FILE).getroot()

    transport = root.find(
        "fastdds:transport_descriptors/fastdds:transport_descriptor",
        NAMESPACE,
    )
    publisher = root.find("fastdds:publisher", NAMESPACE)
    subscriber = root.find("fastdds:subscriber", NAMESPACE)

    assert transport is not None
    assert int(
        transport.findtext("fastdds:receiveBufferSize", namespaces=NAMESPACE)
    ) >= 16 * 1024 * 1024
    assert publisher is not None
    assert publisher.get("is_default_profile") == "true"
    assert subscriber is not None
    assert subscriber.get("is_default_profile") == "true"
    assert root.find("fastdds:data_writer", NAMESPACE) is None
    assert root.find("fastdds:data_reader", NAMESPACE) is None
    assert (
        subscriber.findtext("fastdds:historyMemoryPolicy", namespaces=NAMESPACE)
        == "DYNAMIC_REUSABLE"
    )
