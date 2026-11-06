"""Execute the actual accepted Run tail against deterministic phase costs.

This is a stage-order regression, NOT an estimator replay or a measured latency
benchmark. The C++ control flow is extracted verbatim from laser_mapping.cc;
only expensive phase implementations and publishers are passive test doubles.
The downstream schedule uses the production causal inertial predictor.
"""
import argparse
import hashlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
SOURCE = PACKAGE / "src/laser_mapping.cc"
BEFORE = None
EVIDENCE = None

CPP = r'''
#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
enum class LocalizationRegistrationDisposition { Accepted, WeakGeometry, NonfiniteFilter };
struct KF { int get_P() const { return 42; } };
struct Cloud { std::vector<int> points{1, 2, 3}; };
struct Map { int version=0; int NumValidGrids() const { return version; } };
struct Timer {
  template<typename F> static void Evaluate(F &&fn, const char*) { fn(); }
};
struct Fixture;
struct Logger {
  Fixture *owner;
  template<typename T> Logger &operator<<(const T&) { return *this; }
  ~Logger();
};
#define LOG(x) Logger{this}
struct Fixture {
  bool robust_mode_=true, run_in_offline_=false;
  std::atomic<bool> robust_fault_{false}, robust_imu_clock_reset_{false};
  bool robust_coast_pending_=true, robust_recovering_=true;
  int robust_coast_recoveries_=0, robust_min_features_=10, effect_feat_num_=11;
  int state_point_=23;
  KF kf_;
  double lidar_end_time_=100.1, robust_last_good_=100.0;
  std::string robust_reason_="previous";
  int pub_odom_aft_mapped_=1, pub_path_=1, pub_laser_cloud_body_=1;
  int pub_laser_cloud_effect_world_=1;
  bool pcd_save_en_=true, path_save_en_=true, path_pub_en_=true;
  bool scan_pub_en_=true, scan_body_pub_en_=true, scan_effect_pub_en_=true;
  std::shared_ptr<Cloud> scan_undistort_=std::make_shared<Cloud>();
  std::shared_ptr<Map> ivox_=std::make_shared<Map>();
  int frame_num_=0;
  std::int64_t phase_ns=0;
  bool inject_clock_fault_in_map=false;
  LocalizationRegistrationDisposition quality=LocalizationRegistrationDisposition::Accepted;
  json events=json::array();
  json payload() const {
    return {{"stamp_ns", 100100000000LL}, {"epoch", 7},
      {"frame", "d1max_loc_odom"}, {"child_frame", "d1max_loc_tracking"},
      {"position", {1.0, 2.0, 0.5}}, {"orientation", {0.0, 0.0, 0.0, 1.0}},
      {"world_velocity", {0.3, 0.0, 0.0}}, {"state", state_point_},
      {"covariance", kf_.get_P()}, {"scan_end", lidar_end_time_}};
  }
  void record(const std::string &name, bool valid=false) {
    events.push_back({{"name", name}, {"phase_ns", phase_ns}, {"valid", valid},
      {"payload", payload()}, {"reason", robust_reason_},
      {"fault", robust_fault_.load() || robust_imu_clock_reset_.load()},
      {"map_version", ivox_->version}});
  }
  LocalizationRegistrationDisposition AssessLocalizationRegistration(int,int,int,int) {
    record("quality"); return quality;
  }
  void PublishRobustSample(bool valid) { record("local_sample", valid); }
  void PublishOdometry(int) {
    record("odometry");
    if (robust_mode_)
      PublishRobustSample(!robust_fault_.load() && !robust_imu_clock_reset_.load());
  }
  void LogFrontendState() { record("diagnostic_begin"); phase_ns += 20000000; }
  void MapIncremental() {
    record("map_begin"); phase_ns += 150000000; ++ivox_->version;
    if (inject_clock_fault_in_map) { robust_imu_clock_reset_=true; record("clock_fault"); }
  }
  void PublishPath(int) { record("path"); }
  void PublishFrameWorld() { record("world_cloud"); }
  void PublishFrameBody(int) { record("body_cloud"); }
  void PublishFrameEffectWorld(int) { record("effect_cloud"); }
  void accepted_tail() {
    int cur_pts=3;
@@TAIL@@
  }
  json result() const {
    return {{"events", events}, {"frame_num", frame_num_},
      {"recoveries", robust_coast_recoveries_}, {"last_good", robust_last_good_},
      {"phase_ns", phase_ns}, {"payload_final", payload()},
      {"fault", robust_fault_.load() || robust_imu_clock_reset_.load()}};
  }
};
Logger::~Logger() { owner->record("mapping_log"); owner->phase_ns += 10000000; }
int main() {
  json result;
  for (const std::string name : {"live", "offline", "mapping", "no_publisher", "weak", "nonfinite", "fault_before", "clock_before", "clock_during_map"}) {
    Fixture f;
    if (name=="offline") f.run_in_offline_=true;
    if (name=="mapping") f.robust_mode_=false;
    if (name=="no_publisher") f.pub_odom_aft_mapped_=0;
    if (name=="weak") f.quality=LocalizationRegistrationDisposition::WeakGeometry;
    if (name=="nonfinite") f.quality=LocalizationRegistrationDisposition::NonfiniteFilter;
    if (name=="fault_before") f.robust_fault_=true;
    if (name=="clock_before") f.robust_imu_clock_reset_=true;
    if (name=="clock_during_map") f.inject_clock_fault_in_map=true;
    f.accepted_tail();
    result[name]=f.result();
  }
  std::cout << result.dump(2) << '\n';
}
'''


def accepted_tail(source):
    text = source.read_text()
    run_start = text.index("void LaserMapping::Run() {")
    start = text.index("    if(robust_mode_) {\n        const auto quality=AssessLocalizationRegistration", run_start)
    end = text.index("\n}\n\n// void LaserMapping::StandardPCLCallBack", start)
    block = text[start:end]
    assert block.count("frame_num_++;") == 1
    return block


def execute_tail(source):
    source = source.resolve()
    block = accepted_tail(source)
    with tempfile.TemporaryDirectory(prefix="faster_lio_delivery_") as directory:
        cpp = pathlib.Path(directory) / "tail.cpp"
        binary = pathlib.Path(directory) / "tail"
        cpp.write_text(CPP.replace("@@TAIL@@", block))
        command = ["/usr/bin/g++", "-std=c++17", "-O2", "-pthread", str(cpp), "-o", str(binary)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        result = json.loads(subprocess.run([str(binary)], check=True, capture_output=True, text=True).stdout)
    return {"source": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "tail_sha256": hashlib.sha256(block.encode()).hexdigest(), "cases": result,
            "compile_command": command,
            "scope": "Actual production accepted-tail control flow, passive phase costs; not a native LIO timing replay."}


def named(case, event):
    return [entry for entry in case["events"] if entry["name"] == event]


def posterior(stamp, delivered, valid=True, fault=False, reason="tracking", epoch=7):
    covariance = [0.01 if i // 6 == i % 6 else 0.0 for i in range(36)]
    return dict(schema=1, epoch=epoch, valid=valid, fault=fault, reason=reason,
                received_at_unix=delivered, stamp_ns=round(stamp * 1e9),
                frame="d1max_loc_odom", child_frame="d1max_loc_tracking",
                position=[0.3 * (stamp - 100.0), 0.0, 0.5], orientation=[0.0, 0.0, 0.0, 1.0],
                pose_covariance=covariance, twist_covariance=covariance,
                imu_quality_scope="last_admitted_scan", imu_quality_scan_end_sec=stamp,
                last_good_posterior_sec=stamp, finite_filter_state=True,
                inertial=dict(schema=1, world_velocity=[0.3, 0.0, 0.0], gyro_bias=[0.0]*3,
                              accel_bias=[0.0]*3, gravity=[0.0, 0.0, -9.81], accel_scale=1.0))


def scheduled_predictor(delivery_ns, *, imu_end=100.50, publish_notice=False):
    # Imports are production numerical classes, no ROS runtime or fake filter.
    sys.path.insert(0, str(WORKSPACE / "src/d1max_localization"))
    from d1max_localization.estimation.causal_prediction import CausalInertialPredictor
    from d1max_localization.estimation.prediction import PredictionLimits
    p = CausalInertialPredictor(PredictionLimits(max_horizon=.35, max_coast=.1, max_imu_gap=.1))
    events = [(100.08, "posterior", posterior(100.0, 100.08)),
              (100.20 + delivery_ns * 1e-9, "posterior", posterior(100.1, 100.20 + delivery_ns * 1e-9))]
    events += [(100.0 + i * .005, "imu", 100.0 + i * .005)
               for i in range(101) if 100.0 + i * .005 <= imu_end + 1e-9]
    if publish_notice:
        events.append((100.40, "posterior", posterior(100.1, 100.40, valid=False)))
    events += [(100.08 + i * .02, "tick", None) for i in range(22)]
    result = []
    for when, kind, value in sorted(events, key=lambda e: (e[0], {"imu":0, "posterior":1, "tick":2}[e[1]])):
        if kind == "posterior":
            accepted = p.accept(value, when)
            if not accepted:
                raise AssertionError(f"posterior delivery rejected: {when}, {value}")
        elif kind == "imu":
            if not p.push_imu(value, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0), when):
                raise AssertionError("fixture IMU rejected")
        else:
            state = p.evaluate(when)
            result.append(dict(now=when, valid=state is not None, reason=p.reason,
                               posterior_source=p.snapshot[0] if p.snapshot else None,
                               position=list(state.pose.position) if state else None,
                               velocity=list(state.world_velocity) if state else None,
                               imu_source=state.imu_stamp if state else None))
    return result


class FrontendDeliveryOrder(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.after = execute_tail(SOURCE)
        cls.before = execute_tail(BEFORE) if BEFORE else None
        cls.records = {"after": cls.after, "before": cls.before}

    def test_accepted_posterior_delivered_once_before_maintenance(self):
        case = self.after["cases"]["live"]
        odometry, sample = named(case, "odometry"), named(case, "local_sample")
        self.assertEqual(len(odometry), 1)
        self.assertEqual(len(sample), 1)
        self.assertTrue(sample[0]["valid"])
        names = [event["name"] for event in case["events"]]
        self.assertLess(names.index("quality"), names.index("odometry"))
        self.assertLess(names.index("local_sample"), names.index("diagnostic_begin"))
        self.assertLess(names.index("local_sample"), names.index("map_begin"))
        self.assertEqual(sample[0]["reason"], "tracking")
        self.assertEqual(case["last_good"], 100.1)
        self.assertEqual(case["recoveries"], 1)
        self.assertEqual(case["frame_num"], 1)

    def test_stage_fixture_keeps_original_payload_and_single_delivery(self):
        case = self.after["cases"]["live"]
        sample = named(case, "local_sample")[0]
        self.assertEqual(sample["payload"], case["payload_final"])
        self.assertEqual(sample["payload"]["stamp_ns"], 100100000000)
        self.assertEqual(sample["payload"]["epoch"], 7)
        self.assertEqual(sample["payload"], named(case, "odometry")[0]["payload"])
        if self.before:
            old_sample = named(self.before["cases"]["live"], "local_sample")[0]
            self.assertEqual(sample["payload"], old_sample["payload"])

    def test_rejected_quality_never_publishes_or_inserts_posterior(self):
        for name, expected_fault in (("weak", False), ("nonfinite", True)):
            with self.subTest(name=name):
                case = self.after["cases"][name]
                self.assertEqual(named(case, "odometry"), [])
                self.assertEqual(named(case, "map_begin"), [])
                sample = named(case, "local_sample")
                self.assertEqual(len(sample), 1)
                self.assertFalse(sample[0]["valid"])
                self.assertEqual(sample[0]["fault"], expected_fault)
                self.assertEqual(case["frame_num"], 0)
                self.assertEqual(case["last_good"], 100.0)

    def test_atomic_fault_before_delivery_never_publishes_valid_source(self):
        for name in ("fault_before", "clock_before"):
            with self.subTest(name=name):
                samples = named(self.after["cases"][name], "local_sample")
                self.assertEqual(len(samples), 1)
                self.assertFalse(samples[0]["valid"])
                self.assertTrue(samples[0]["fault"])

    def test_no_second_valid_posterior_after_clock_fault_during_map(self):
        case = self.after["cases"]["clock_during_map"]
        self.assertTrue(case["fault"])
        fault_at = named(case, "clock_fault")[0]["phase_ns"]
        samples = named(case, "local_sample")
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]["valid"], samples[0]["phase_ns"] < fault_at)
        self.assertFalse(any(s["valid"] and s["phase_ns"] >= fault_at for s in samples))

    def test_offline_mapping_absent_publisher_unchanged(self):
        for name in ("offline", "mapping", "no_publisher"):
            with self.subTest(name=name):
                case = self.after["cases"][name]
                self.assertEqual(named(case, "local_sample"), [])
                if name != "mapping":
                    self.assertEqual(named(case, "odometry"), [])
                else:
                    self.assertEqual(named(case, "odometry")[0]["phase_ns"], 180000000)
                if self.before:
                    self.assertEqual(case, self.before["cases"][name])

    def test_world_cloud_retains_map_update_dependency(self):
        for case in self.after["cases"].values():
            for cloud in named(case, "world_cloud"):
                self.assertEqual(cloud["map_version"], 1)

    def test_synthetic_phase_counterfactual_same_posterior(self):
        if not self.before:
            self.skipTest("--before needed for same-source historical comparison")
        after = named(self.after["cases"]["live"], "local_sample")[0]
        before = named(self.before["cases"]["live"], "local_sample")[0]
        self.assertEqual(before["phase_ns"] - after["phase_ns"], 180000000)
        self.assertEqual(self.before["cases"]["live"]["phase_ns"], self.after["cases"]["live"]["phase_ns"])

    def test_production_predictor_source_horizon_no_longer_consumed_by_maintenance(self):
        after_ns = named(self.after["cases"]["live"], "local_sample")[0]["phase_ns"]
        after = scheduled_predictor(after_ns)
        self.records["predictor_after"] = after
        self.assertTrue(all(row["valid"] for row in after if .0 <= row["now"] - 100.08 <= .36))
        for row in after:
            if row["valid"]:
                self.assertAlmostEqual(row["velocity"][0], .3, places=10)
                self.assertAlmostEqual(row["position"][0], .3*(row["now"]-100.0), places=9)
        if self.before:
            before_ns = named(self.before["cases"]["live"], "local_sample")[0]["phase_ns"]
            before = scheduled_predictor(before_ns)
            self.records["predictor_before"] = before
            at_36 = next(row for row in before if abs(row["now"]-100.36)<1e-8)
            self.assertFalse(at_36["valid"])
            self.assertEqual(at_36["reason"], "lio_stale")
            self.assertTrue(next(row for row in after if abs(row["now"]-100.36)<1e-8)["valid"])

    def test_original_horizon_not_renewed_by_tracking_notice(self):
        delivery = named(self.after["cases"]["live"], "local_sample")[0]["phase_ns"]
        result = scheduled_predictor(delivery, publish_notice=True)
        self.records["tracking_notice_expiry"] = result
        expired = next(row for row in result if abs(row["now"]-100.46)<1e-8)
        self.assertFalse(expired["valid"])
        self.assertEqual(expired["reason"], "lio_stale")
        self.assertAlmostEqual(expired["posterior_source"], 100.1)

    def test_true_imu_outage_still_expires(self):
        delivery = named(self.after["cases"]["live"], "local_sample")[0]["phase_ns"]
        result = scheduled_predictor(delivery, imu_end=100.24)
        self.records["imu_outage_negative"] = result
        # Both old/new phase orders have received the new posterior by .40;
        # source-horizon priority cannot mask the independent IMU outage here.
        missing = next(row for row in result if abs(row["now"]-100.40)<1e-8)
        self.assertFalse(missing["valid"])
        self.assertEqual(missing["reason"], "imu_stale")
        self.assertAlmostEqual(missing["posterior_source"], 100.1)

    @classmethod
    def tearDownClass(cls):
        if EVIDENCE:
            EVIDENCE.write_text(json.dumps(cls.records, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=pathlib.Path, default=SOURCE)
    parser.add_argument("--before", type=pathlib.Path)
    parser.add_argument("--evidence", type=pathlib.Path)
    args, test_args = parser.parse_known_args()
    SOURCE, BEFORE, EVIDENCE = args.source, args.before, args.evidence
    unittest.main(argv=[sys.argv[0], *test_args])
