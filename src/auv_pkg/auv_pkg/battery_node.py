#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, time, math, signal, threading
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from std_msgs.msg import String, Float32, Bool

# I2C backends
try:
    from adafruit_extended_bus import ExtendedI2C  # preferred on Pi
    _HAS_EXTENDED = True
except Exception:
    _HAS_EXTENDED = False
    ExtendedI2C = None  # type: ignore

try:
    import board, busio
except Exception:
    board = None
    busio = None

# INA260
from adafruit_ina260 import INA260

# ------------------------- LiPo SoC helper -------------------------
def lipo_3s_soc_from_voltage(v_pack: float) -> float:
    """
    Very simple OCV-based SoC for 3S LiPo (approx at rest / light load).
    Uses a piecewise curve; clamps to [0,100].
    """
    v_cell = v_pack / 3.0
    # Quick piecewise approximation (idle):
    # 4.20V -> 100%
    # 4.00V -> ~85%
    # 3.85V -> ~60%
    # 3.70V -> ~30%
    # 3.50V -> ~10%
    # 3.30V -> ~0%
    pts = [
        (3.30, 0.0),
        (3.50, 10.0),
        (3.70, 30.0),
        (3.85, 60.0),
        (4.00, 85.0),
        (4.20, 100.0),
    ]
    if v_cell <= pts[0][0]:
        return 0.0
    if v_cell >= pts[-1][0]:
        return 100.0
    for (vx, sx), (vy, sy) in zip(pts, pts[1:]):
        if vx <= v_cell <= vy:
            t = (v_cell - vx) / (vy - vx)
            return max(0.0, min(100.0, sx + t * (sy - sx)))
    return 0.0

# ------------------------ Persistence helpers -----------------------
STATE_PATH = Path.home() / ".local/share/auv_battery_node.json"
STATE_VERSION = 2  # bump if units/format change

def load_state() -> dict:
    try:
        if STATE_PATH.exists():
            with STATE_PATH.open("r") as f:
                data = json.load(f)
            if data.get("version") == STATE_VERSION:
                return data
    except Exception:
        pass
    return {"version": STATE_VERSION, "packs": {}}

def save_state(state: dict) -> None:
    try:
        STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        tmp = STATE_PATH.with_suffix(".tmp")
        with tmp.open("w") as f:
            json.dump(state, f)
        tmp.replace(STATE_PATH)
    except Exception:
        pass

# ----------------------------- Pack --------------------------------
class Pack:
    def __init__(self, name: str, addr: int, capacity_mAh: float, i2c_bus_id: int, node: Node):
        self.name = name
        self.addr = addr
        self.capacity_mAh = capacity_mAh
        self.node = node

        self.present = False
        self.ina: Optional[INA260] = None
        self.i2c_bus_id = i2c_bus_id
        self._i2c = None

        # dynamics
        self.used_mAh = 0.0
        self.used_Wh = 0.0
        self.last_ts = None
        self._needs_baseline = True

        # behavior-based estimates (simple moving average over N seconds)
        self.window_sec = int(max(60, node.get_parameter("behavior_window_sec").value))
        self._avg_lock = threading.Lock()
        self._p_hist = []  # (timestamp, power_W)

        # pubs
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        base = f"/battery/{name}"

        self.pub_summary      = node.create_publisher(String , f"{base}/summary", qos_transient)
        self.pub_present      = node.create_publisher(Bool   , f"{base}/present", qos_transient)
        self.pub_voltage      = node.create_publisher(Float32, f"{base}/voltage_V", 10)
        self.pub_current      = node.create_publisher(Float32, f"{base}/current_A", 10)
        self.pub_power        = node.create_publisher(Float32, f"{base}/power_W"  , 10)
        self.pub_soc          = node.create_publisher(Float32, f"{base}/soc_pct"  , 10)
        self.pub_used_mAh     = node.create_publisher(Float32, f"{base}/used_mAh" , 10)
        self.pub_used_Wh      = node.create_publisher(Float32, f"{base}/used_Wh"  , 10)
        self.pub_eta_min      = node.create_publisher(Float32, f"{base}/eta_min"  , 10)  # behavior-based
        self.pub_est_topic    = node.create_publisher(String , f"{base}/behavior_based_estimation", 10)

    # --------- I2C bring-up ----------
    def _new_i2c(self):
        if _HAS_EXTENDED:
            return ExtendedI2C(self.i2c_bus_id)
        if busio is None or board is None:
            raise RuntimeError("No I2C backend available")
        return busio.I2C(board.SCL, board.SDA)

    def ensure_sensor(self):
        """Try to (re)acquire sensor if not present."""
        if self.present:
            return
        try:
            if self._i2c is None:
                self._i2c = self._new_i2c()
            self.ina = INA260(self._i2c, address=self.addr)
            # sanity read
            _ = self.ina.voltage
            self.present = True
            self._needs_baseline = True
            self.last_ts = None
            self.node.get_logger().info(f"[INA260/{self.name}] online @0x{self.addr:02X} on bus {self.i2c_bus_id}")
            self.pub_present.publish(Bool(data=True))
        except Exception as e:
            # stay non-blocking
            self.present = False
            self.ina = None
            self.last_ts = None
            # only log occasionally
            self.node.get_logger().warn(f"[INA260/{self.name}] not ready: {e}")
            self.pub_present.publish(Bool(data=False))

    # --------- Unit-correct sampling & integration ----------
    def sample(self, t_now: float) -> Optional[Tuple[float, float, float, float]]:
        """
        Returns (V, I_A, P_W, SoC%) or None if not present / read error.
        Integrates used_mAh / used_Wh with correct units.
        """
        if not self.present:
            self.ensure_sensor()
            if not self.present:
                return None
        assert self.ina is not None

        try:
            v_V  = float(self.ina.voltage)      # volts
            i_mA = float(self.ina.current)      # milliamps
            p_mW = float(self.ina.power)        # milliwatts

            # Convert to A / W consistently
            i_A = i_mA / 1000.0
            p_W = p_mW / 1000.0

            if self._needs_baseline:
                self._apply_ocv_baseline(v_V)
                self._needs_baseline = False

            # Integrate with correct units
            if self.last_ts is None:
                self.last_ts = t_now
            dt = max(0.0, t_now - self.last_ts)
            self.last_ts = t_now

            # Accumulate consumption (only discharge; clamp regen/noise)
            if i_mA > 0:
                self.used_mAh += (i_mA * dt) / 3600.0
            if p_W > 0:
                self.used_Wh  += (p_W * dt) / 3600.0

            # Compute SoC by coulomb counting blended with OCV hint at low load
            soc_coul = max(0.0, 100.0 * (1.0 - (self.used_mAh / max(1.0, self.capacity_mAh))))
            # Simple blend: if power < 2W, weight OCV 40%; else mostly coulomb
            soc_ocv = lipo_3s_soc_from_voltage(v_V)
            w = 0.4 if p_W < 2.0 else 0.1
            soc = max(0.0, min(100.0, (1.0 - w) * soc_coul + w * soc_ocv))

            # Behavior-based estimate (moving avg power ➜ ETA)
            with self._avg_lock:
                self._p_hist.append((t_now, p_W))
                # drop old
                cutoff = t_now - self.window_sec
                while self._p_hist and self._p_hist[0][0] < cutoff:
                    self._p_hist.pop(0)
                avg_p = sum(p for _, p in self._p_hist) / max(1, len(self._p_hist))
            # Remaining Wh estimated from capacity & used_Wh (capacity from mAh ~ Vnom*Ah)
            # Use 3S nominal 11.1V to convert capacity_mAh to Wh baseline:
            pack_Wh_nom = (self.capacity_mAh / 1000.0) * 11.1
            used_Wh_capped = max(0.0, min(pack_Wh_nom, self.used_Wh))
            rem_Wh = max(0.0, pack_Wh_nom - used_Wh_capped)
            eta_min = float("inf")
            if avg_p > 1e-3:
                eta_min = (rem_Wh / avg_p) * 60.0

            # Publish raw channels
            self.pub_voltage.publish(Float32(data=v_V))
            self.pub_current.publish(Float32(data=i_A))
            self.pub_power.publish(Float32(data=p_W))
            self.pub_soc.publish(Float32(data=soc))
            self.pub_used_mAh.publish(Float32(data=self.used_mAh))
            self.pub_used_Wh.publish(Float32(data=self.used_Wh))
            if math.isfinite(eta_min):
                self.pub_eta_min.publish(Float32(data=eta_min))
                self.pub_est_topic.publish(String(data=f"{self.name}: avgP~{avg_p:.2f}W ETA~{eta_min:.1f}min"))

            # Summary (human)
            summary = String()
            summary.data = (f"{self.name}: V={v_V:.2f}V I={i_A:.3f}A P~{p_W:.2f}W "
                            f"SoC={soc:.1f}% used={self.used_mAh:.0f}mAh/{self.capacity_mAh:.0f}mAh")
            self.pub_summary.publish(summary)
            return v_V, i_A, p_W, soc

        except Exception as e:
            self.node.get_logger().warn(f"[INA260/{self.name}] read error: {e}")
            # mark absent and retry later
            self.present = False
            self.pub_present.publish(Bool(data=False))
            return None

    def _apply_ocv_baseline(self, v_pack: float) -> None:
        """Align counters to the estimated SoC derived from the present voltage."""
        soc_ocv = lipo_3s_soc_from_voltage(v_pack)
        frac_used = max(0.0, min(1.0, 1.0 - (soc_ocv / 100.0)))

        cap_mAh = max(1.0, float(self.capacity_mAh))
        pack_wh_nom = (cap_mAh / 1000.0) * 11.1

        # Update counters so coulomb counting starts from the observed state of charge
        self.used_mAh = frac_used * cap_mAh
        self.used_Wh = frac_used * pack_wh_nom

        self.node.get_logger().info(
            f"[INA260/{self.name}] baseline from OCV {soc_ocv:.1f}% -> used {self.used_mAh:.0f}mAh"
        )

# ----------------------------- Node --------------------------------
class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")

        # Parameters
        self.declare_parameter("i2c_bus_id", 1)
        self.declare_parameter("compute_addr", 0x45)  # both A0/A1 bridged
        self.declare_parameter("actuation_addr", 0x44)  # A1 bridged only
        self.declare_parameter("compute_capacity_mAh", 5300.0)
        self.declare_parameter("actuation_capacity_mAh", 5300.0)
        self.declare_parameter("poll_hz", 5.0)
        self.declare_parameter("behavior_window_sec", 300)  # 5 minutes

        self._bus_id = int(self.get_parameter("i2c_bus_id").value)
        comp_addr = int(self.get_parameter("compute_addr").value)
        act_addr  = int(self.get_parameter("actuation_addr").value)
        comp_cap  = float(self.get_parameter("compute_capacity_mAh").value)
        act_cap   = float(self.get_parameter("actuation_capacity_mAh").value)
        poll_hz   = max(0.5, float(self.get_parameter("poll_hz").value))
        self._period = 1.0 / poll_hz

        # Packs
        self.compute = Pack("compute", comp_addr, comp_cap, self._bus_id, self)
        self.actuation = Pack("actuation", act_addr, act_cap, self._bus_id, self)
        self._packs = [self.compute, self.actuation]

        # Restore persisted counters (with version guard)
        state = load_state()
        for p in self._packs:
            s = state.get("packs", {}).get(p.name, {})
            # Only accept sane values (avoid old 1000x scaled data)
            um = float(s.get("used_mAh", 0.0))
            uw = float(s.get("used_Wh", 0.0))
            if um > 0 and um < 100000.0:  # reject obviously-bad old scale
                p.used_mAh = um
            if uw > 0 and uw < 10000.0:
                p.used_Wh = uw

        # Persistence timer (30s)
        self.create_timer(30.0, self._persist)

        # Sampling timer
        self.create_timer(self._period, self._tick)

        # Param updates live
        self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info(
            f"[BatteryNode] bus={self._bus_id} poll={poll_hz:.1f}Hz | compute@0x{comp_addr:02X}({comp_cap:.0f}mAh) "
            f"actuation@0x{act_addr:02X}({act_cap:.0f}mAh)"
        )

    # --------- timers ----------
    def _tick(self):
        t = time.time()
        for p in self._packs:
            p.sample(t)

    def _persist(self):
        state = {"version": STATE_VERSION, "packs": {}}
        for p in self._packs:
            state["packs"][p.name] = {
                "used_mAh": p.used_mAh,
                "used_Wh": p.used_Wh,
            }
        save_state(state)

    # --------- params ----------
    def _on_param_update(self, params):
        changed = []
        for prm in params:
            if prm.name == "compute_capacity_mAh" and prm.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                self.compute.capacity_mAh = float(prm.value); changed.append(prm.name)
            elif prm.name == "actuation_capacity_mAh" and prm.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                self.actuation.capacity_mAh = float(prm.value); changed.append(prm.name)
            elif prm.name == "behavior_window_sec" and prm.type_ in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
                val = int(prm.value)
                self.compute.window_sec = self.actuation.window_sec = max(60, val)
                changed.append(prm.name)
        if changed:
            self.get_logger().info(f"[BatteryNode] updated params: {', '.join(changed)}")
        return rclpy.parameter.SetParametersResult(successful=True)

# ----------------------------- main --------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    try:
        ex.spin()  # Ctrl-C raises KeyboardInterrupt here
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info("[BatteryNode] Shutting down...")
            node._persist()           # save counters one last time
        except Exception:
            pass
        try:
            ex.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
