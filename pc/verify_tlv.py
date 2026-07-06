"""Verify the new range-antenna TLV (type 8) on live hardware."""
import sys, time
sys.path.insert(0, '/Users/sady3721/project/owl/Spatial3D/pc')
from spatial3d import uart_reader as U
from spatial3d.tlv import TLV_RANGE_ANTENNA

CLI  = '/dev/cu.usbmodem0000RA441'
DATA = '/dev/cu.usbmodem0000RA444'
CFG  = '/Users/sady3721/project/TI/Tiinstall/profile_4T4R_verify.cfg'

print("=== 1) 发 cfg 到 CLI（看回显 / 报错）===")
U.send_config(CLI, CFG, echo=True)

print("\n=== 2) 读 DATA 帧，找 range-antenna TLV(type 8)===")
import serial
ser = U.open_serial(DATA, U.DATA_BAUD, timeout=3.0)
from spatial3d.tlv import read_frame
n=0; hit=0; first=None
t0=time.time()
try:
    while n < 40 and time.time()-t0 < 20:
        f = read_frame(ser)
        n += 1
        ra = f.range_antenna()
        types = [t.type for t in f.tlvs]
        if ra is not None and ra.num_bins > 0:
            hit += 1
            if first is None:
                first = ra
                print(f"[frame {f.header.frame_number}] TLVs={types}  RANGE-ANTENNA: start_bin={ra.start_bin} num_bins={ra.num_bins} shape={ra.data.shape} dtype={ra.data.dtype}")
                print(f"  bin0 前3天线复数: {ra.data[0,:3]}")
        elif n <= 3:
            print(f"[frame {f.header.frame_number}] TLVs={types} (无 range-antenna)")
finally:
    ser.close()

print(f"\n=== 结果: 读了 {n} 帧, {hit} 帧含 range-antenna TLV ===")
print("✅ 新固件 + 新 TLV 工作正常!" if hit>0 else "⚠️ 没收到 range-antenna TLV — 看上面 CLI 回显是否有 Error")
