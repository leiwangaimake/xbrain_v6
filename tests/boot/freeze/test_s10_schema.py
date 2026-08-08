"""CFG-FZ-17 S10 schema-validation tests: t_lat_s variant + baseline."""

import pytest

from xbrain.boot.freeze.assertions.s10_schema import run
from xbrain.common.errors.exceptions import XbrainError


def _ctx(file_trees=None, skip=None):
    return {
        "config_root": "/tmp",
        "file_trees": file_trees or {},
        "skip_files": skip or (),
    }


def test_green_brake_yaml_passes():
    """safety/brake.yaml with numeric t_lat_s/a_mps2/k passes."""
    tree = {"common": {"safety": {"t_lat_s": 0.4,
                                   "brake": {"a_mps2": 2.5, "k": 1.2}}}}
    result = run(_ctx({"safety/brake.yaml": tree}))
    assert result["status"] == "pass"
    assert result["files_checked"] == 1


# CFG-FZ-17 variant 1: t_lat_s as string
def test_variant_1_t_lat_s_string_is_red():
    tree = {"common": {"safety": {"t_lat_s": "0.4",  # STRING not number
                                   "brake": {"a_mps2": 2.5, "k": 1.2}}}}
    with pytest.raises(XbrainError) as ei:
        run(_ctx({"safety/brake.yaml": tree}))
    assert ei.value.detail["kind"] == "schema_validation_failed"
    assert ei.value.detail["file"] == "safety/brake.yaml"


def test_wrong_type_bool_where_number_expected():
    tree = {"common": {"safety": {"t_lat_s": True,  # BOOL not number
                                   "brake": {"a_mps2": 2.5, "k": 1.2}}}}
    with pytest.raises(XbrainError) as ei:
        run(_ctx({"safety/brake.yaml": tree}))
    assert ei.value.detail["kind"] == "schema_validation_failed"


def test_missing_required_field_is_red():
    tree = {"common": {"safety": {"t_lat_s": 0.4,
                                   "brake": {"a_mps2": 2.5}}}}  # k missing
    with pytest.raises(XbrainError) as ei:
        run(_ctx({"safety/brake.yaml": tree}))
    assert ei.value.detail["kind"] == "schema_validation_failed"


def test_clock_all_optional_empty_passes():
    """safety/clock.yaml: every field required=False -> empty passes."""
    result = run(_ctx({"safety/clock.yaml": {}}))
    assert result["status"] == "pass"


def test_clock_wrong_type_bool_expected_int_given():
    tree = {"common": {"safety": {"clock": {"rtc_trusted": 1}}}}
    with pytest.raises(XbrainError) as ei:
        run(_ctx({"safety/clock.yaml": tree}))
    assert ei.value.detail["kind"] == "schema_validation_failed"


def test_unregistered_file_is_red():
    tree = {"foo": "bar"}
    with pytest.raises(XbrainError) as ei:
        run(_ctx({"random/scratch.yaml": tree}))
    assert ei.value.detail["kind"] == "schema_unregistered_file"


def test_skip_files_bypasses_validation():
    tree = {"common": {"safety": {"t_lat_s": "0.4"}}}  # would fail if not skipped
    result = run(_ctx({"safety/brake.yaml": tree},
                      skip=["safety/brake.yaml"]))
    assert result["status"] == "pass"
    assert result["files_checked"] == 0


def test_empty_overrides_walks_no_files():
    result = run({"config_root": "/tmp/does-not-exist"})
    assert result["status"] == "pass"


def test_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
