from pathlib import Path

import pytest
import yaml

from lightweight_sim.tests.configuration import load_configuration


@pytest.fixture(scope="session")
def installed_configuration():
    from ament_index_python.packages import get_package_share_directory

    return load_configuration(get_package_share_directory("lightweight_sim") + "/config")


@pytest.fixture(scope="session")
def acceptance_configuration():
    return yaml.safe_load(
        Path(__file__).with_name("acceptance.yaml").read_text(encoding="utf-8")
    )
