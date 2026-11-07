"""Host-only original-target/control equivalence; requires a clang-compatible C compiler path."""
from pathlib import Path
import subprocess,sys,xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[3]
out=root/'tools/matlab/mpc10/output/host';out.mkdir(parents=True,exist_ok=True)
s=(root/'imcalib/Algorithm/lqr/lqr_balance.c').read_text(encoding='utf-8')
a=s.index('    /* 只发布原目标采用的静态工作点');b=s.index('    /* 位置参考',a)
(out/'lqr_before_working_point.c').write_text(s[:a]+s[b:],encoding='utf-8')
t=ET.parse(root/'MDK-ARM/CtrBoard-H7_ALL.uvprojx');inc=next(e.text for e in t.iter('IncludePath') if e.text and 'Algorithm/lqr/inc' in e.text)
compiler=sys.argv[1];flags=[]
for x in inc.split(';'):flags+=['-I'+str((root/'MDK-ARM'/x).resolve())]
names=['LQR_Init','LQR_Velocity_Apply','LQR_Enable_Latch','LQR_Target_Update','LQR_State_Update','LQR_Control_Update','lqr_debug']
subprocess.run([compiler,'cc','-O2','-UNDEBUG']+flags+['-D'+n+'='+n+'_Baseline' for n in names]+['-c',str(out/'lqr_before_working_point.c'),'-o',str(out/'lqr_before_working_point.o')],check=True)
subprocess.run([compiler,'cc','-O2','-UNDEBUG']+flags+[str(root/'tools/matlab/mpc10/host_lqr_operating_point_test.c'),str(out/'lqr_before_working_point.o'),str(root/'imcalib/Algorithm/lqr/lqr_balance.c'),str(root/'imcalib/common/simple-function.c'),'-o',str(out/'lqr_working_point_test.exe')],check=True)
subprocess.run([str(out/'lqr_working_point_test.exe')],check=True)
