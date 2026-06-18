import yaml
from pathlib import Path
from yolo26.prepare_dataset import write_dataset_yaml


def test_escribe_yaml_valido(tmp_path):
    out = write_dataset_yaml(tmp_path / "duckietown.yaml", dataset_dir=tmp_path / "ds")
    assert out.exists()
    data = yaml.safe_load(Path(out).read_text())
    assert data["names"] == {0: "duckie", 1: "cone", 2: "truck", 3: "bus"}
    assert data["path"].endswith("ds")
