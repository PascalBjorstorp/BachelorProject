"""Regression checks for the localhost calibration operator interface."""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import termios
import threading
import time
import unittest
import urllib.request
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT))

from calibration_suite.guidance import STAGE_GUIDANCE  # noqa: E402
from calibration_suite.gui import (  # noqa: E402
    CalibrationGuiServer,
    GuiController,
    TerminalJob,
    validate_metrology,
)
from calibration_suite.runner import STAGES  # noqa: E402


class CalibrationGuiTests(unittest.TestCase):
    def test_main_pc_launcher_builds_a_private_password_free_plan(self) -> None:
        result = subprocess.run(
            [sys.executable, str(REPO / "run_test"), "10.126.128.198", "--dry-run"],
            cwd=REPO, text=True, capture_output=True, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("127.0.0.1:8765:127.0.0.1:8765", result.stdout)
        self.assertIn("f1tenth@10.126.128.198", result.stdout)
        self.assertNotIn("admin321", result.stdout)

    def test_every_stage_has_operator_guidance(self) -> None:
        self.assertEqual({stage.key for stage in STAGES}, set(STAGE_GUIDANCE))
        for stage in STAGES:
            guide = STAGE_GUIDANCE[stage.key]
            for key in (
                "title", "summary", "category", "abc_role", "setup_figure",
                "setup", "during", "expect", "outputs", "data",
            ):
                self.assertTrue(guide.get(key), f"{stage.key} has no {key}")

    def test_prefilled_measurements_do_not_fake_missing_axle_metrology(self) -> None:
        template = yaml.safe_load((ROOT / "config" / "physical_measurements.yaml").read_text(encoding="utf-8"))
        result = validate_metrology(template)
        self.assertFalse(result["ready"])
        self.assertIn("required.front_axle_load_N.value", result["errors"])
        self.assertIn("required.rear_axle_load_N.value", result["errors"])
        self.assertFalse(any("stddev" in error for error in result["errors"]))

    def test_old_measurement_sheets_are_presented_and_saved_as_value_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            controller = GuiController(
                ROOT / "config" / "suite.yaml", REPO,
                runs_root=Path(temporary) / "runs",
            )
            session = controller.create_session()
            path = session / "physical_measurements.yaml"
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            document["required"]["mass_kg"]["stddev"] = 0.01
            document["required"]["mass_kg"]["method"] += "; confirm and enter one-sigma uncertainty"
            document["measurement_context"]["notes"] = "enter the missing uncertainties and axle-scale readings"
            path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

            presented = controller.snapshot()["metrology"]["document"]
            self.assertNotIn("stddev", json.dumps(presented))
            self.assertNotIn("uncert", json.dumps(presented).lower())

            controller.update_metrology({"measurement_context": {"operator": "migration test"}})
            saved = yaml.safe_load(path.read_text(encoding="utf-8"))
            self.assertNotIn("stddev", json.dumps(saved))
            self.assertNotIn("uncert", json.dumps(saved).lower())

    def test_old_stage_sequences_are_inspect_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            controller = GuiController(
                ROOT / "config" / "suite.yaml", REPO,
                runs_root=Path(temporary) / "runs",
            )
            session = controller.create_session()
            manifest_path = session / "session_manifest.yaml"
            manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
            manifest["stage_order"] = manifest["stage_order"][:-1]
            manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")

            status = controller.snapshot()
            self.assertFalse(status["manifest"]["compatible"])
            self.assertEqual(status["manifest"]["total"], len(STAGES) - 1)
            self.assertTrue(all(not stage["can_run"] for stage in status["stages"]))
            session_row = next(row for row in status["sessions"] if row["active"])
            self.assertFalse(session_row["compatible"])
            self.assertEqual(session_row["total"], len(STAGES) - 1)
            with self.assertRaisesRegex(RuntimeError, "older test sequence"):
                controller.run_stage("steering_command_audit", [])

    def test_controller_saves_form_and_confines_result_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            controller = GuiController(
                ROOT / "config" / "suite.yaml", REPO,
                runs_root=Path(temporary) / "runs",
            )
            session = controller.create_session()
            total = 3.314 * 9.80665
            validation = controller.update_metrology({
                "required": {
                    "mass_kg": {"value": 3.314},
                    "wheelbase_m": {"value": 0.324},
                    "front_axle_load_N": {"value": 16.0},
                    "rear_axle_load_N": {"value": total - 16.0},
                },
                "measurement_context": {"operator": "GUI test"},
            })
            self.assertTrue(validation["ready"])
            status = controller.snapshot()
            self.assertEqual(status["next_stage"], "steering_command_audit")
            self.assertEqual(len(status["stages"]), 28)
            self.assertTrue(status["metrology"]["validation"]["ready"])
            self.assertEqual(status["metrology"]["document"]["measurement_context"]["operator"], "GUI test")
            with self.assertRaisesRegex(ValueError, "accepts only a value"):
                controller.update_metrology({"required": {"mass_kg": {"stddev": 0.01}}})
            with self.assertRaisesRegex(ValueError, "missing safety confirmations"):
                controller.run_stage("steering_command_audit", [])
            self.assertFalse(controller.job.running)
            command = controller._runner_command("run", "--stage", "steering_command_audit")
            self.assertNotIn("sh", command)
            self.assertIn(str(session), command)

            artifact = session / "analysis" / "safe.json"
            artifact.parent.mkdir(parents=True, exist_ok=True)
            artifact.write_text(json.dumps({"ok": True}), encoding="utf-8")
            self.assertEqual(controller.safe_session_file("analysis/safe.json"), artifact)
            with self.assertRaises(PermissionError):
                controller.safe_session_file("../session_manifest.yaml")
            with self.assertRaises(FileNotFoundError):
                controller.safe_session_file("physical_measurements.yaml.bag")

    def test_terminal_job_supports_line_and_single_key_prompts(self) -> None:
        code = (
            "import sys,termios,tty\n"
            "assert input('Type READY: ').strip() == 'READY'\n"
            "print('KEY MODE [a/d z/c x/v l r q]', flush=True)\n"
            "fd=sys.stdin.fileno(); old=termios.tcgetattr(fd)\n"
            "try:\n"
            " tty.setraw(fd); key=sys.stdin.read(1)\n"
            "finally:\n"
            " termios.tcsetattr(fd, termios.TCSADRAIN, old)\n"
            "print('KEY='+key, flush=True)\n"
        )
        job = TerminalJob()
        job.start([sys.executable, "-u", "-c", code], cwd=REPO, label="pty-test")
        self._wait_for_log(job, "Type READY")
        job.write("READY")
        self._wait_for_log(job, "KEY MODE")
        job.write("x", newline=False)
        deadline = time.monotonic() + 5.0
        while job.running and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertFalse(job.running)
        deadline = time.monotonic() + 1.0
        while job.snapshot()["status"] == "running" and time.monotonic() < deadline:
            time.sleep(0.01)
        snapshot = job.snapshot()
        self.assertEqual(snapshot["status"], "succeeded")
        self.assertIn("KEY=x", snapshot["log"])

    def test_http_health_endpoint(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            controller = GuiController(
                ROOT / "config" / "suite.yaml", REPO,
                runs_root=Path(temporary) / "runs",
            )
            session = controller.create_session()
            plot = session / "plots" / "stages" / "steering_command_audit.png"
            plot.parent.mkdir(parents=True, exist_ok=True)
            plot.write_bytes(b"\x89PNG\r\n\x1a\noperator-plot")
            pdf = session / "analysis" / "vehicle_calibration_report.pdf"
            pdf.parent.mkdir(parents=True, exist_ok=True)
            pdf.write_bytes(b"%PDF-1.4\noperator-report")
            server = CalibrationGuiServer(("127.0.0.1", 0), controller)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                port = server.server_address[1]
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/health", timeout=3.0) as response:
                    payload = json.loads(response.read().decode("utf-8"))
                self.assertTrue(payload["ok"])
                self.assertEqual(payload["service"], "vehicle-calibration-gui")

                with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status", timeout=3.0) as response:
                    status = json.loads(response.read().decode("utf-8"))
                self.assertEqual(len(status["artifacts"]["plots"]), 1)
                plot_url = status["artifacts"]["plots"][0]["url"]
                with urllib.request.urlopen(f"http://127.0.0.1:{port}{plot_url}", timeout=3.0) as response:
                    self.assertEqual(response.headers.get_content_type(), "image/png")
                    self.assertTrue(response.read().startswith(b"\x89PNG"))
                with urllib.request.urlopen(
                    f"http://127.0.0.1:{port}{status['artifacts']['pdf']}", timeout=3.0,
                ) as response:
                    self.assertEqual(response.headers.get_content_type(), "application/pdf")
                    self.assertTrue(response.read().startswith(b"%PDF"))
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2.0)

    @staticmethod
    def _wait_for_log(job: TerminalJob, text: str) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if text in job.snapshot()["log"]:
                return
            time.sleep(0.02)
        raise AssertionError(f"did not see terminal prompt {text!r}: {job.snapshot()['log']!r}")


if __name__ == "__main__":
    unittest.main()
