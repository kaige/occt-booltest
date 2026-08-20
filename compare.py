import csv, glob, os

def load(basedir):
    d = {}
    for f in sorted(glob.glob(basedir + '/iterations/*_iterations.csv')):
        rows = list(csv.DictReader(open(f)))
        if not rows: continue
        case = rows[0]['case']
        last = rows[-1]
        d[case] = dict(iters=len(rows), status=last['status'], V=float(last['volume']),
                       pct=last['volume_pct_of_V0'], faces=int(last['faces']),
                       nsucc=sum(1 for r in rows if r['status']=='success'),
                       nfail=sum(1 for r in rows if r['status']=='failed'))
    return d

old  = load('output')             # 旧内存跑（原二进制）
new  = load('output_memregress')  # 新内存回归
step = load('output_step')        # STEP 模式

def terms(basedir):
    t = {}
    try:
        for r in csv.DictReader(open(basedir + '/summary.csv')):
            t[r['case']] = r['termination']
    except FileNotFoundError: pass
    return t
told, tnew, tstep = terms('output'), terms('output_memregress'), terms('output_step')

order = sorted(set(old) | set(new) | set(step))
def fmt(x):
    return f"{float(x):.4g}" if x else "-"

print(f"{'case':<17} | {'旧内存':>18} | {'新内存回归':>18} | {'STEP模式':>18}")
print('-'*76)
for c in order:
    def cell(d, t):
        if c not in d: return '缺失'
        r = d[c]
        mark = 'OK ' if r['nfail']==0 else 'BAD'
        return f"{mark}{r['nsucc']}/{t.get(c,'?')[:10]}/{fmt(r['pct'])}"
    print(f"{c:<17} | {cell(old,told):>18} | {cell(new,tnew):>18} | {cell(step,tstep):>18}")

print()
print("== 回归一致性检查（旧内存 vs 新内存）==")
diff = []
common = sorted(set(old).intersection(set(new)))
for c in common:
    o, n = old[c], new[c]
    if o['nsucc'] != n['nsucc'] or fmt(o['pct']) != fmt(n['pct']):
        diff.append((c, o['nsucc'], n['nsucc'], o['pct'], n['pct']))
if diff:
    for c,a,b,p1,p2 in diff: print(f"  差异: {c}: {a}轮/{p1}% -> {b}轮/{p2}%")
else:
    print(f"  全部 {len(common)} 案例逐项一致（迭代数+终体积）")

print()
print("== 模式 flip 统计（内存失败/成功 之间翻转的案例）==")
flips = []
common2 = sorted(set(old).intersection(set(step)))
for c in common2:
    o, s = old[c], step[c]
    ofail = o['nfail']>0 or float(o['pct'] or 0)==0
    sfail = s['nfail']>0 or float(s['pct'] or 0)==0
    if ofail != sfail:
        tag = '败->成' if not sfail else '成->败'
        flips.append(f"  {c}: {tag}  ({o['nsucc']}轮/{fmt(o['pct'])}% -> {s['nsucc']}轮/{fmt(s['pct'])}%)")
print('\n'.join(flips) if flips else "  无翻转")

print()
print("== 体积/面数漂移（两模式都成功的案例）==")
for c in common2:
    o, s = old[c], step[c]
    if o['nfail']==0 and s['nfail']==0 and float(o['pct'])>0 and float(s['pct'])>0:
        po, ps = float(o['pct']), float(s['pct'])
        drift = (ps-po)/po*100 if po else 0
        fdrift = (s['faces']-o['faces'])
        print(f"  {c}: V {fmt(o['pct'])}% -> {fmt(s['pct'])}% (漂移{drift:+.1f}%)  faces {o['faces']} -> {s['faces']} ({fdrift:+d})")

print()
print("== STEP 模式失败明细 ==")
for c in common2:
    s = step[c]
    if s['nfail']>0:
        print(f"  {c}: 第{s['nsucc']+1}轮失败")
