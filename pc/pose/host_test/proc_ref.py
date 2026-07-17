import numpy as np, subprocess, re, sys, os
HERE=os.path.dirname(os.path.abspath(__file__))
PC=os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, PC)
from pose.model import folded_forward

# exported weights
src=open(os.path.join(PC, "..", "firmware", "people_tracking_6844", "src", "6844",
                      "mss", "source", "pose", "pose_model.c")).read()
def grab(name):
    m=re.search(r"const float %s((?:\[\d+\])+) = \{(.*?)\};"%name,src,re.S)
    dims=[int(d) for d in re.findall(r"\[(\d+)\]",m.group(1))]
    vals=np.array([float(x[:-1]) for x in re.findall(r"[-+][0-9.]+e[-+][0-9]+f",m.group(2))])
    return vals.reshape(dims)
layers=[(grab(f"gPoseW{i}"),grab(f"gPoseB{i}")) for i in range(4)]

GATE=0.75; NF=8; NP=5; ZOFF=0.30
rng=np.random.default_rng(7)

def build_frame(kin, pts):
    # kin: dict; pts: (M,4) x,y,z,snr
    d=pts[:,:2]-np.array([kin['posX'],kin['posY']])
    g=pts[(d[:,0]**2+d[:,1]**2)<=GATE*GATE]
    if len(g)<NP: return None
    order=np.argsort(g[:,2],kind='stable')      # ascending z
    top=g[order][-NP:]                           # 5 highest, ascending
    f=[kin['posZ']+ZOFF, kin['velY'], kin['velZ'], kin['accY'], kin['accZ']]
    for r in top:
        f += [r[1]-kin['posY'], r[2], r[3]]
    return np.array(f,dtype=np.float32)

# scripted sequence: one track, 10 frames; enough points each
lines=[f"{ZOFF}"]
ref_windows=[]   # list of (frame_index, expected feature 160-vec) once buffer full
hist=[]
for fi in range(10):
    kin=dict(tid=1,
             posX=float(rng.uniform(-0.5,0.5)), posY=float(rng.uniform(2,3)),
             posZ=float(rng.uniform(-1.5,0.8)), velY=float(rng.normal()*0.3),
             velZ=float(rng.normal()*0.3), accY=float(rng.normal()*0.3), accZ=float(rng.normal()*0.3))
    M=rng.integers(6,12)
    pts=np.column_stack([rng.uniform(kin['posX']-0.5,kin['posX']+0.5,M),
                         rng.uniform(kin['posY']-0.5,kin['posY']+0.5,M),
                         rng.uniform(-1.6,1.0,M), rng.uniform(5,40,M)]).astype(np.float32)
    lines.append(f"F 1 {M}")
    lines.append(f"1 {kin['posX']} {kin['posY']} {kin['posZ']} {kin['velY']} {kin['velZ']} {kin['accY']} {kin['accZ']}")
    for p in pts: lines.append(f"{p[0]} {p[1]} {p[2]} {p[3]}")
    fr=build_frame(kin,pts)
    hist.append(fr)
    # replicate ring: only pushes when fr is not None
    pushed=[h for h in hist if h is not None]
    if len(pushed)>=NF:
        win=np.stack(pushed[-NF:])               # (8,20) k=0 oldest
        flat=win.T.reshape(-1)                    # feature-major -> 160
        ref_windows.append((fi, flat))

seqf=os.path.join(HERE, "_proc_seq.txt")
open(seqf,"w").write("\n".join(lines)+"\n")
PROC=os.environ.get("POSE_PROC","pose_proc_test")
out=subprocess.run([PROC, seqf],capture_output=True,text=True)
frames=[b for b in out.stdout.split("---\n") if b.strip()]
print(f"C emitted {len(frames)} frame-blocks; ref expects valid inference on {len(ref_windows)} frames")
ok=True
ri=0
for fi in range(10):
    blk=frames[fi].strip().split("\n") if fi<len(frames) else []
    cline=blk[0].split() if blk else []
    cvalid=int(cline[3]) if len(cline)==4 else -1
    expect_valid = any(w[0]==fi for w in ref_windows)
    if expect_valid:
        _,flat=ref_windows[ri]; ri+=1
        prob=folded_forward(layers,flat.astype(np.float64))
        exp_pose=int(prob.argmax()); exp_fp=int(prob[3]*255+0.5)
        cpose=int(cline[1]); cfp=int(cline[2])
        match = (cvalid==1 and cpose==exp_pose and abs(cfp-exp_fp)<=1)
        ok &= match
        print(f"  frame {fi}: C(pose={cpose},fp={cfp},valid={cvalid}) ref(pose={exp_pose},fp={exp_fp}) {'ok' if match else 'MISMATCH'}")
    else:
        if cvalid==1: ok=False; print(f"  frame {fi}: C valid but ref expected invalid  MISMATCH")
print("\nRESULT:", "MATCH" if ok else "MISMATCH")
