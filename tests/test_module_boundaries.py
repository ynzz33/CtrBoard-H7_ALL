"""Keep algorithms independent of task state and compensation layers separate."""
from pathlib import Path
import re
import unittest

import test_structure_runtime as runtime

ROOT = Path(__file__).resolve().parents[1]


class ModuleBoundariesTest(unittest.TestCase):
    def test_algorithms_do_not_include_task_state_or_task_interfaces(self):
        for path in (ROOT / "imcalib/Algorithm").glob("*.[ch]"):
            with self.subTest(file=path.name):
                includes = re.findall(r'^#include\s+"([^"]+)"', path.read_text(encoding="utf-8"), re.M)
                for include in includes:
                    self.assertNotIn(Path(include).name, ("robot_control.h", "robot_state.h"))

    def test_task_interface_does_not_export_shared_state(self):
        source = (ROOT / "imcalib/task/inc/robot_control.h").read_text(encoding="utf-8")
        self.assertNotIn('#include "robot_state.h"', source)
        self.assertNotRegex(source, r'extern\s+(?:volatile\s+)?(?:imu_state_t|motor_state_t|leg_state_t|lqr_state_t)\b')
        self.assertNotIn('#include "lqr_balance.h"', source)

    def test_gravity_model_does_not_orchestrate_spring_or_output_limits(self):
        source = (ROOT / "imcalib/Algorithm/gravity_comp.c").read_text(encoding="utf-8")
        self.assertNotIn("Gas_Spring_Apply", source)
        self.assertNotIn("Leg_Compensation_Apply", source)
        self.assertNotIn("compensation_debug_t", source)

    def test_leg_controller_does_not_read_global_runtime_switches(self):
        source = (ROOT / "imcalib/Algorithm/leg_balance.c").read_text(encoding="utf-8")
        self.assertNotRegex(source, r'\blqr_debug\b')

    def test_pure_data_and_control_headers_compile_without_platform_stubs(self):
        # Real headers: no fake main.h, HAL, RTOS or CubeAI declarations.
        runtime.StructureRuntimeTest.run_c(self, r'''
#include "motor_types.h"
#include "rc_types.h"
#include "rl_types.h"
#include "feedback_snapshot.h"
#include "torque_output.h"
#include "rl_torque.h"
#include "lqr_balance.h"
#include "leg_balance.h"
#include "standup.h"
#include "air_detection.h"
int main(void) { return 0; }
''')

    def test_default_task_owns_background_services(self):
        comm = (ROOT / "imcalib/task/task_comm.c").read_text(encoding="utf-8")
        default = (ROOT / "Core/Src/freertos.c").read_text(encoding="utf-8")
        background = (ROOT / "imcalib/task/background_service.c").read_text(encoding="utf-8")
        self.assertNotRegex(comm, r'Vofa_|WS2812_')
        body = default[default.index("void StartDefaultTask(", default.index("void StartDefaultTask(") + 1):]
        self.assertLess(body.index("MX_USB_DEVICE_Init();"), body.index("background_task_body();"))
        self.assertLess(body.index("background_task_body();"), body.index("osDelay(1);"))
        self.assertLess(background.index("Vofa_Debug_Tick();"), background.index("WS2812_RainbowBlink();"))


if __name__ == "__main__":
    unittest.main()
