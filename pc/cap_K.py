import sys, os, time, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spatial3d.range_music import DR_M, N_VIRT_ANT, BinAccumulator
from spatial3d.uart_reader import RadarSession
CLI="/dev/cu.usbmodem0000RA441"; DATA="/dev/cu.usbmodem0000RA444"
CFG="/Users/sady3721/project/TI/Tiinstall/profile_music_5fps_fullroom.cfg"
K=100
s=RadarSession(CLI,DATA); s.start_drain()
live=0; t0=time.time()
while time.time()-t0<6 and live<5:
    f=s.get_frame(timeout=1.0)
    if f is not None and f.range_antenna() is not None: live+=1
if live>=5:
    print("attached, settle 30s",flush=True); t0=time.time()
    while time.time()-t0<30: s.get_frame(timeout=0.5)
else:
    print("cfg+preheat 120s",flush=True); s.send_cfg(CFG,echo=False); t0=time.time()
    while time.time()-t0<120: s.get_frame(timeout=0.5)
acc=BinAccumulator(k=K,n_ant=N_VIRT_ANT); bins=range(87,271)
print(f"capturing evK (lie @4m, k={K})...",flush=True); t0=time.time()
while time.time()-t0<90 and acc.min_count(bins)<K:
    f=s.get_frame(timeout=1.0)
    if f is None: continue
    ra=f.range_antenna()
    if ra is not None: acc.add(ra)
s.close()
common=[b for b in bins if len(acc.snaps.get(b,[]))>=10]
cov=np.stack([(np.stack(acc.snaps[b]).conj().T@np.stack(acc.snaps[b]))/len(acc.snaps[b]) for b in common]).astype(np.complex64)
mean=np.stack([np.stack(acc.snaps[b]).mean(0) for b in common]).astype(np.complex64)
np.savez('evK.npz',bins=np.array(common,dtype=np.int32),cov=cov,mean=mean,dr_m=np.float32(DR_M))
print(f"SAVED evK.npz {len(common)} bins, min/bin={acc.min_count(bins)}",flush=True)
