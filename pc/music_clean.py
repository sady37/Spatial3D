import sys, time, threading
sys.path.insert(0,'/Users/sady3721/project/owl/Spatial3D/pc')
from spatial3d import uart_reader as U
from spatial3d.tlv import read_frame
CLI='/dev/cu.usbmodem0000RA441'; DATA='/dev/cu.usbmodem0000RA444'
CFG='/Users/sady3721/project/TI/Tiinstall/profile_4T4R_music.cfg'

# 先读 CLI banner 确认设备刚复位、活着
cli=U.open_serial(CLI,U.CLI_BAUD,timeout=0.5)
time.sleep(0.3); banner=cli.read(2048).decode(errors='replace')
print("=== CLI banner(应有 'MMW Demo') ===\n"+ (banner.strip() or "(空 — 可能没复位/CLI无输出)"))

data=U.open_serial(DATA,U.DATA_BAUD,timeout=1.0)
st={'ra':None,'n':0,'stop':False,'err':''}
def drain():
    while not st['stop']:
        try:
            f=read_frame(data); st['n']+=1
            ra=f.range_antenna()
            if ra and ra.num_bins>0: st['ra']=(ra.start_bin,ra.num_bins,ra.data.shape)
        except Exception: pass
threading.Thread(target=drain,daemon=True).start()

def send(line,wait):
    cli.reset_input_buffer(); cli.write((line+'\n').encode())
    t0=time.time();buf=''
    while time.time()-t0<wait:
        d=cli.read(512).decode(errors='replace')
        if d:buf+=d
        if 'Done' in buf or 'Error' in buf:break
    flag='OK' if 'Done' in buf else ('ERR:'+buf.replace(line,'').strip()[:120] if 'Error' in buf else 'NO-Done')
    print(f"> {line:38s} [{flag}]")
    if 'Error' in buf: st['err']+=line+' | '
    return buf

lines=[l.strip() for l in open(CFG) if l.strip() and not l.strip().startswith('%')]
for l in lines:
    send(l, 5.0 if l.startswith('sensorStart') else 1.5)
print("\n=== 等 3 秒看 DATA ===")
time.sleep(3)
print(f"最新 range-antenna: {st['ra']}  (总帧 {st['n']})")
sb=st['ra'][0] if st['ra'] else None
if sb==87: print("✅ music profile 生效! start_bin=87, ΔR≈2.34cm")
elif sb==20: print("⚠️ 还是 start_bin=20 → music cfg 没生效")
else: print(f"⚠️ start_bin={sb}, 报错行: {st['err'] or '无'}")
# 收尾:停流,避免再卡
st['stop']=True; time.sleep(0.3)
cli.reset_input_buffer(); cli.write(b'sensorStop 0\n'); time.sleep(0.5)
data.close(); cli.close()
