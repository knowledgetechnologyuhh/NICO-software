import pypot.dynamixel
import time

from nicomotion import Motion
from os.path import abspath, dirname, join

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError("No ports found")

with pypot.dynamixel.DxlIO(ports[0]) as dxl_io:
    scan_range = range(50)
    print(f"scanning ids in {scan_range}...")
    ids = dxl_io.scan(scan_range)
    print(f"detected ids: {ids}")
    models = dxl_io.get_model(ids)
    firmware_versions = dxl_io.get_firmware(ids)
    for i, model, firmware in zip(ids, models, firmware_versions):
        print("-------------------------------")
        print(f"id: {i}")
        print(f"model: {model}")
        print(f"firmware: {firmware}")
