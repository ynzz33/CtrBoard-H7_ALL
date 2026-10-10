"""Check shared motor command timing at the driver boundary."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from test_gas_spring_integration import ROOT, production_function, read


class RefactorHelpersTest(unittest.TestCase):
    def run_host(self, source):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("set CC to host gcc")
        env = os.environ.copy()
        env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
        with tempfile.TemporaryDirectory(prefix="refactor-helpers-") as tmp:
            folder = Path(tmp)
            host = folder / "helpers.c"
            host.write_text(source, encoding="utf-8")
            exe = folder / "helpers.exe"
            result = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "imcalib/drivers"), "-I", str(ROOT / "imcalib/bsp"), "-I", str(ROOT / "imcalib/common"), "-I", str(ROOT / "imcalib/config"), "-I", str(ROOT / "imcalib/task/inc"), str(host), "-o", str(exe)],
                capture_output=True, text=True, env=env)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_finite_array_validation_rejects_null_nan_and_infinity(self):
        self.run_host(r'''
#include <assert.h>
#include <float.h>
#include "math_utils.h"
int main(void)
{
    float values[4]={0,-0.0f,FLT_MAX,-FLT_MAX};
    const float invalid[3]={NAN,INFINITY,-INFINITY};
    float saved;
    unsigned i,j;
    assert(!Float_Array_Finite(NULL,0) && !Float_Array_Finite(NULL,4));
    assert(Float_Array_Finite(values,0) && Float_Array_Finite(values,4));
    for(i=0;i<4;i++) {
        saved=values[i];
        for(j=0;j<3;j++) {
            values[i]=invalid[j];assert(!Float_Array_Finite(values,4));
        }
        values[i]=saved;
    }
    assert(Float_Array_Finite(values,4));return 0;
}
''')

    def test_motor_retries_keep_independent_timers_and_try_every_motor(self):
        source = r'''
#include <assert.h>
#include <stdint.h>
#include "motor_types.h"
#define DM_CMD_ENABLE 0xFCu
#define DM_CMD_DISABLE 0xFDu
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
static struct { uint8_t err_raw; } dm_motor_feedback[4];
static uint8_t online[4] = {1,1,1,1},command[4];
static unsigned sends[4];
static uint32_t tick;
static uint32_t HAL_GetTick(void) { return tick; }
static uint8_t Dm_Is_Online(uint8_t i) { return online[i]; }
static HAL_StatusTypeDef Dm_Send_Command(uint8_t i,uint8_t cmd)
{ sends[i]++;command[i]=cmd;return i==1 ? HAL_ERROR : HAL_OK; }
'''
        production = read("imcalib/drivers/dm.c")
        for name in ("Dm_Command_Watchdog", "Dm_Enable_Watchdog", "Dm_Disable_Watchdog",
                     "Dm_All_Command", "Dm_All_Enable", "Dm_All_Disable"):
            source += "\n" + production_function(production, name)
        source += r'''
int main(void)
{
    unsigned i;
    tick=99;Dm_Enable_Watchdog();assert(!sends[0]);
    online[2]=0;dm_motor_feedback[3].err_raw=8;
    tick=100;Dm_Enable_Watchdog();
    assert(sends[0]==1 && sends[1]==1 && !sends[2] && !sends[3]);
    tick=199;Dm_Enable_Watchdog();assert(sends[0]==1);
    tick=200;Dm_Enable_Watchdog();assert(sends[0]==2 && sends[1]==2);
    dm_motor_feedback[0].err_raw=1;Dm_Disable_Watchdog();
    assert(sends[0]==3 && command[0]==DM_CMD_DISABLE);
    tick=299;Dm_Disable_Watchdog();assert(sends[0]==3);
    tick=300;Dm_Disable_Watchdog();assert(sends[0]==4);
    dm_motor_feedback[0].err_raw=0;
    tick=UINT32_MAX-49;Dm_Enable_Watchdog();assert(sends[0]==5);
    tick=49;Dm_Enable_Watchdog();assert(sends[0]==5);
    tick=50;Dm_Enable_Watchdog();assert(sends[0]==6);
    assert(Dm_All_Enable()==HAL_ERROR);
    for(i=0;i<4;i++) {assert(command[i]==DM_CMD_ENABLE && sends[i]);}
    assert(Dm_All_Disable()==HAL_ERROR);
    for(i=0;i<4;i++) {assert(command[i]==DM_CMD_DISABLE);}
    assert(sends[0]==8 && sends[1]==6 && sends[2]==2 && sends[3]==2);
    return 0;
}
'''
        self.run_host(source)
