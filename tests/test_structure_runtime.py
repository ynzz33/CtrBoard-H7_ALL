"""Run production snapshot and LED code with deterministic preemption/I/O doubles."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import ROOT, read, without_includes, production_function


class StructureRuntimeTest(unittest.TestCase):
    def run_c(self, text):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("set CC to host gcc")
        env = os.environ.copy()
        env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
        with tempfile.TemporaryDirectory(prefix="structure-runtime-") as tmp:
            folder = Path(tmp)
            source = folder / "runtime.c"
            source.write_text(text, encoding="utf-8")
            exe = folder / "runtime.exe"
            args = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "imcalib/drivers"), "-I", str(ROOT / "imcalib/bsp"), "-I", str(ROOT / "imcalib/common"), "-I", str(ROOT / "imcalib/config"), "-I", str(ROOT / "imcalib/task/inc"), "-I", str(ROOT / "imcalib/Algorithm"),
                    "-I", str(ROOT / "imcalib/task/inc"), str(source), "-o", str(exe)]
            result = subprocess.run(args, capture_output=True, text=True, env=env)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True, env=env, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_snapshot_publication_metadata_and_preemption(self):
        prefix = r'''
#include <assert.h>
#include <string.h>
#include <limits.h>
#include "feedback_snapshot.h"
static imu_state_t imu_state;
static motor_state_t motor_state,next_motor;
static leg_state_t leg_l,leg_r,next_left,next_right;
static rc_command_t rc_command;
static struct { uint8_t rc_enable; } robot_state;
static struct { uint8_t mode; } input_command;
static feedback_sample_t samples[6];
static uint64_t clock_ns;
static unsigned critical,pending;
static void enter(void) { critical++; }
static void leave(void) {
    assert(critical);critical--;
    if(!critical && pending) {
        pending=0;
        Robot_Feedback_Publish_Motor(&next_motor,&next_left,&next_right,samples);
    }
}
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
static uint64_t Mono_Ns_Get(void) { return clock_ns; }
'''
        production = without_includes(read("imcalib/task/feedback_snapshot.c"))
        # Attempt a task switch at the boundary between motor and leg copying.
        production = production.replace("out->motor = motor_state;", "out->motor = motor_state; if(pending) { assert(critical); }")
        self.run_c(prefix + production + r'''
int main(void) {
    robot_feedback_t frame;
    feedback_sample_t received={0,1};
    assert(!Robot_Feedback_Read(NULL));
    assert(Robot_Feedback_Read(&frame) && !frame.imu_meta.seen);
    imu_state.online=1;imu_state.last_timestamp_ms=123;
    Robot_Feedback_Publish_Imu(&imu_state,received);
    Robot_Feedback_Publish_Rc(&rc_command,1,3,received);
    motor_state.timestamp_ms=1;leg_l.output.virtual_leg_length=.1f;
    leg_r.output.virtual_leg_length=.2f;
    samples[0]=received;
    Robot_Feedback_Publish_Motor(&motor_state,&leg_l,&leg_r,samples);
    next_motor.timestamp_ms=2;next_left.output.virtual_leg_length=.3f;
    next_right.output.virtual_leg_length=.4f;pending=1;clock_ns=999;
    assert(Robot_Feedback_Read(&frame));
    assert(frame.motor.timestamp_ms==1 && frame.left.output.virtual_leg_length==.1f);
    assert(frame.right.output.virtual_leg_length==.2f && motor_state.timestamp_ms==2);
    assert(frame.imu.last_timestamp_ms==123 && frame.imu_sample.seen);
    assert(frame.imu_sample.received_ns==0 && frame.captured_ns==999);
    assert(robot_state.rc_enable && input_command.mode==3);
    clock_ns=2000;assert(Robot_Feedback_Read(&frame));
    assert(frame.motor.timestamp_ms==2 && frame.left.output.virtual_leg_length==.3f);
    assert(frame.imu_sample.received_ns==0 && frame.imu_meta.sequence==1);
    metadata.motor.sequence=UINT32_MAX;
    Robot_Feedback_Publish_Motor(&motor_state,&leg_l,&leg_r,samples);
    assert(Robot_Feedback_Read(&frame) && frame.motor_meta.sequence==0 && frame.motor_meta.seen);
    imu_state.online=0;received.seen=0;
    Robot_Feedback_Publish_Imu(&imu_state,received);
    assert(Robot_Feedback_Read(&frame) && !frame.imu.online && frame.imu_sample.seen);
    assert(frame.imu_sample.received_ns==0);
    return 0;
}
''')

    def test_led_encoding_busy_timeout_and_wall_clock_phases(self):
        prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#define WS2812_SPI_UNIT spi
#define HAL_SPI_STATE_READY 0
#define HAL_OK 0
static struct { unsigned State; } spi;
static uint64_t ns;
static uint32_t cost=1000;
static unsigned writes,failed;
static uint8_t first[24];
static uint64_t Mono_Ns_Get(void) { return ns; }
static int HAL_SPI_Transmit(void *handle,const uint8_t *p,unsigned len,unsigned timeout) {
    (void)handle;assert(len && timeout && timeout<=5);
    writes++;if(len==24) {memcpy(first,p,24);}
    if(failed) {return 1;}
    if(cost>(uint64_t)timeout*1000000) {ns+=(uint64_t)timeout*1000000;return 1;}
    ns+=cost;return 0;
}
'''
        self.run_c(prefix + without_includes(read("imcalib/drivers/ws2812.c")) + r'''
int main(void) {
    unsigned i,previous;uint64_t start;
    WS2812_RainbowBlink();assert(writes==101);
    for(i=0;i<8;i++) {assert(first[i]==0x60 && first[8+i]==0x78 && first[16+i]==0x60);}
    ns=250000000;WS2812_RainbowBlink();
    for(i=0;i<24;i++) {assert(first[i]==0x60);}
    previous=writes;ns=300000000;WS2812_RainbowBlink();assert(writes==previous);
    spi.State=1;ns=500000000;WS2812_RainbowBlink();assert(writes==previous);
    spi.State=0;failed=1;WS2812_RainbowBlink();assert(writes==previous+1);
    failed=0;WS2812_RainbowBlink();assert(writes==previous+102);
    cost=2000000;start=ns;assert(!WS2812_Send(1,2,3));assert(ns-start<=5000000);
    spi.State=1;previous=writes;WS2812_Ctrl(0,0,0);assert(writes==previous);
    return 0;
}
''')

    def test_transport_switch_requires_disabled_and_previous_send_complete(self):
        prefix = r'''
#include <assert.h>
#include <stdint.h>
#define VOFA_TRANSPORT_UART 1u
#define VOFA_TRANSPORT_USB 0u
#define HAL_UART_STATE_READY 0u
static struct { unsigned gState; } uart;
#define VOFA_UART (&uart)
static struct { uint8_t requested,active; } vofa_transport={1,1};
static uint8_t usb_ready,usb_idle;
static uint8_t CDC_Transmit_Ready_HS(void) {return usb_ready;}
static uint8_t CDC_Transmit_Idle_HS(void) {return usb_idle;}
'''
        source = read("imcalib/bsp/Vofa_send.c")
        functions = "\n".join(production_function(source, name) for name in
            ("Vofa_Transport_Ready", "Vofa_Transport_Idle", "Vofa_Transport_Update"))
        self.run_c(prefix + functions + r'''
int main(void) {
    vofa_transport.requested=0;
    assert(!Vofa_Transport_Update(0) && vofa_transport.active==1);
    uart.gState=1;assert(!Vofa_Transport_Update(1) && vofa_transport.active==1);
    uart.gState=0;assert(Vofa_Transport_Update(1) && vofa_transport.active==0);
    assert(!Vofa_Transport_Ready());usb_ready=1;assert(Vofa_Transport_Ready());
    vofa_transport.requested=1;
    assert(!Vofa_Transport_Update(1) && vofa_transport.active==0);
    usb_idle=1;assert(!Vofa_Transport_Update(0));assert(Vofa_Transport_Update(1));
    vofa_transport.requested=2;assert(!Vofa_Transport_Update(1) && vofa_transport.active==1);
    return 0;
}
''')
