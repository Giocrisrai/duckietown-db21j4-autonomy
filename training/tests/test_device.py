import yolo26.device as dev


def test_prefer_explicito_se_respeta():
    assert dev.select_device("cpu") == "cpu"


def test_auto_cuda(monkeypatch):
    monkeypatch.setattr(dev, "_cuda_ok", lambda: True)
    monkeypatch.setattr(dev, "_mps_ok", lambda: False)
    assert dev.select_device("auto") == "cuda"


def test_auto_mps(monkeypatch):
    monkeypatch.setattr(dev, "_cuda_ok", lambda: False)
    monkeypatch.setattr(dev, "_mps_ok", lambda: True)
    assert dev.select_device("auto") == "mps"


def test_auto_cpu(monkeypatch):
    monkeypatch.setattr(dev, "_cuda_ok", lambda: False)
    monkeypatch.setattr(dev, "_mps_ok", lambda: False)
    assert dev.select_device("auto") == "cpu"
