"""Python driver against a mocked serial port that behaves like the firmware."""

import pytest

from rada_align.config import load_config
from rada_align.hw.esp32_actuators import ESP32Actuators, SimulatedESP32Port, parse_status
from rada_align.interfaces import ActuatorError, Actuators

CFG = load_config()


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture()
def rig():
    clock = Clock()
    port = SimulatedESP32Port(clock=clock)
    driver = ESP32Actuators(CFG, port=port, keepalive=False)
    return driver, port, clock


def test_driver_is_an_actuators_and_pings_on_connect(rig):
    driver, port, _ = rig
    assert isinstance(driver, Actuators)
    assert port.log == ["PING"]


def test_units_are_converted_to_steps_and_back(rig):
    driver, port, _ = rig
    driver.move_relative("x", 10.0)  # 0.0025 mm per step
    assert port.log[-1] == "MOVE X 4000"
    driver.move_relative("pan", -0.9)  # 0.001125 deg per step
    assert port.log[-1] == "MOVE PAN -800"
    assert driver.position("x") == pytest.approx(10.0)
    assert driver.position("pan") == pytest.approx(-0.9)
    assert driver.position("tilt") == 0.0
    assert not driver.is_moving()
    assert driver.limits("z") == (-250.0, 250.0)


def test_direction_flag_flips_an_axis():
    port = SimulatedESP32Port()
    flipped = CFG.replace_path("hw.direction", (-1, 1, 1, 1))
    driver = ESP32Actuators(flipped, port=port, keepalive=False)
    driver.move_relative("x", 1.0)
    assert port.log[-1] == "MOVE X -400"
    assert driver.position("x") == pytest.approx(1.0)


def test_soft_limits_are_checked_by_driver_and_firmware(rig):
    driver, port, _ = rig
    with pytest.raises(ActuatorError, match="soft limit"):
        driver.move_relative("x", 300.0)
    assert not any(line.startswith("MOVE") for line in port.log)  # never sent
    # Firmware limit tighter than config: the firmware's refusal is reported.
    port.soft_max[0] = 1000
    with pytest.raises(ActuatorError, match="SOFT_LIMIT"):
        driver.move_relative("x", 10.0)
    with pytest.raises(ActuatorError, match="unknown axis"):
        driver.move_relative("yaw", 1.0)


def test_watchdog_stops_motion_until_the_next_ping(rig):
    driver, port, clock = rig
    driver.move_relative("z", 1.0)
    assert driver.position("z") == pytest.approx(1.0)  # first move finished
    clock.t += 1.5  # no PING for longer than the 1 s watchdog
    with pytest.raises(ActuatorError, match="WATCHDOG"):
        driver.move_relative("z", 1.0)
    assert driver.status().watchdog is True
    driver.ping()
    assert driver.status().watchdog is False
    driver.move_relative("z", 1.0)
    assert driver.position("z") == pytest.approx(2.0)


def test_estop_latches_and_needs_release_plus_ping(rig):
    driver, port, _ = rig
    port.estop_button = True
    with pytest.raises(ActuatorError, match="ESTOP"):
        driver.move_relative("x", 1.0)
    assert driver.status().estop is True
    with pytest.raises(ActuatorError, match="ESTOP"):
        driver.ping()  # still pressed
    port.estop_button = False
    assert driver.status().estop is True  # latched until a PING after release
    driver.ping()
    assert driver.status().estop is False
    driver.move_relative("x", 1.0)


def test_timeouts_and_garbage_become_actuator_errors(rig):
    driver, port, _ = rig
    port.silent = True
    with pytest.raises(ActuatorError, match="timeout"):
        driver.stop()
    port.silent = False
    port._replies.append(b"\xff\xfegarbage\n")
    with pytest.raises(ActuatorError, match="unexpected reply"):
        driver.ping()
    with pytest.raises(ActuatorError, match="cannot parse"):
        parse_status("OK pos=1,2,3 moving=000")


def test_home_stop_speed_and_led(rig):
    driver, port, _ = rig
    driver.move_relative("tilt", 2.0)
    driver.home("tilt")
    assert port.log[-1] == "HOME TILT" and driver.position("tilt") == 0.0
    assert driver.status().homed["tilt"] is True
    driver.stop()
    assert port.log[-1] == "STOP"
    driver.set_speed("x", 20.0, 40.0)  # mm/s -> steps/s
    assert port.log[-1] == "SPEED X 8000.0 16000.0"
    with pytest.raises(ActuatorError, match="OUT_OF_RANGE"):
        driver.set_speed("x", 100.0, 40.0)
    driver.set_led(True)
    assert port.led == "ON"
    driver.set_led(False)
    assert port.led == "OFF"
    driver.blink_led(5)
    assert port.led == "BLINK 5"
    driver.close()
    assert port.closed


def test_keepalive_thread_feeds_the_watchdog_and_reports_a_dead_link():
    import time

    port = SimulatedESP32Port(watchdog_s=0.2)
    fast = CFG.replace_path("hw.ping_interval_s", 0.03)
    driver = ESP32Actuators(fast, port=port, keepalive=True)
    time.sleep(0.4)  # longer than the watchdog
    driver.move_relative("x", 1.0)  # still allowed: the thread kept pinging
    assert port.log.count("PING") >= 5
    port.silent = True
    time.sleep(0.15)
    with pytest.raises(ActuatorError, match="keep-alive failed"):
        driver.move_relative("x", 1.0)
    port.silent = False
    driver.close()
