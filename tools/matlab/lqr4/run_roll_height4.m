function varargout = run_roll_height4(machine_name)
%RUN_ROLL_HEIGHT4 鍥涚姸鎬丩QR鍞竴璋冨弬/鍑鸿〃鍏ュ彛锛岀粨鏋勫搴攍qr10/run_all銆?
% 鐩存帴杩愯锛歳un_roll_height4锛涘彲閫塠ok,result]=run_roll_height4銆?
% 妫€鏌ラ€氳繃鍚庣敓鎴愬€欓€夊苟瑕嗙洊鍥轰欢C鏁版嵁锛決/R鍜岃娴嬫椂闂村父鏁板彧鏀硅繖閲屻€?
CFG.machine='big_wheelleg';
CFG.model='roll_height4_lqr';
CFG.deploy=true;
if nargin>=1 && ~isempty(machine_name),CFG.machine=machine_name;end
% 鐘舵€侀『搴忥細roll銆乺oll瑙掗€熷害銆丠銆丠dot锛決/R浠ュ綋鍓嶈皟鍙傚€间负鍑嗭紝鍚嶄箟妫€鏌ラ€氳繃涓嶇瓑浜庡疄鏈虹ǔ瀹氥€?
q=[14 0.3 400 0.01]; % 作者roll角权重20->14；q2=0.5实测放大5Hz且roll峰峰0.60rad，退回0.3
r=[1e-4 1e-4];
observer_tau=[0.02 0.5]; % s锛氱墿鐞嗙姸鎬佷及璁?鎱㈠彉鍔涘亸缃紱鎸夊綋鍓?kHz閲嶆柊璁捐L
addpath(fileparts(mfilename('fullpath')));
[ok,result]=build_roll_height4(CFG,q,r,observer_tau);
if nargout>=1,varargout{1}=ok;end
if nargout>=2,varargout{2}=result;end
end
