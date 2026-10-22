"""Export an offline interactive 3D comparison from recorded demo runs."""

from pathlib import Path
import sys
import json

import numpy as np
import plotly.graph_objects as go
from plotly.offline import get_plotlyjs

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from experiments.joint_risk.export_five_through import (
    COLORS, SHORT, VARIANTS, _source_state, _tick_for, load_variant,
)
from simulation.core.obstacle_motion import obstacle_position


def sphere(center, radius, color, opacity, name):
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 25), np.linspace(0, np.pi, 17))
    return go.Surface(
        x=center[0] + radius * np.cos(u) * np.sin(v),
        y=center[1] + radius * np.sin(u) * np.sin(v),
        z=center[2] + radius * np.cos(v),
        colorscale=[[0, color], [1, color]], showscale=False,
        opacity=opacity, name=name, hoverinfo="name", showlegend=False,
    )


def dynamic_traces(pair, time):
    config = pair["uniform"]["config"]
    result = [sphere(obstacle_position(o, time), o["radius"], "#d94b43", 0.9,
                     f"Physical: {o['name']}") for o in config.obstacles]
    for name in VARIANTS:
        value = pair[name]
        tick = _tick_for(value, time)
        recording = value["recording"]
        means = recording["estimated_obstacle_predictions"][tick, :, 1, :]
        radii = recording["tightened_safety_radius_horizons"][tick, 1, :]
        for j, (center, radius) in enumerate(zip(means, radii)):
            halo_color = "#b44de0" if name == "joint_hard" else "#258df2"
            result.append(sphere(center, radius, halo_color, 0.13,
                                 f"{SHORT[name]}: estimated k=1, obstacle {j + 1}"))
        point = _source_state(value, time)[:3]
        result.append(go.Scatter3d(x=[point[0]], y=[point[1]], z=[point[2]],
                                  mode="markers", marker=dict(size=7, color=COLORS[name]),
                                  name=f"{SHORT[name]} drone", showlegend=False))
    return result


def main():
    pair = {name: load_variant(ROOT / "outputs/joint_risk", name, "five_through", "through_demo")
            for name in VARIANTS}
    export_dashboard(pair)
    return
    traces = []
    for name in VARIANTS:
        value = pair[name]
        positions = np.vstack(([0, 0, 1.2], value["states"][:, :3]))
        traces.append(go.Scatter3d(
            x=positions[:, 0], y=positions[:, 1], z=positions[:, 2], mode="lines",
            line=dict(width=6, color=COLORS[name]), name=SHORT[name],
        ))
    traces.append(go.Scatter3d(x=[0, 3], y=[0, 0], z=[1.2, 1.2], mode="markers+text",
                              marker=dict(size=6, color=["#333333", "#d6a000"]),
                              text=["Start", "Goal"], name="Start / Goal"))
    offset = len(traces)
    traces.extend(dynamic_traces(pair, 0))
    times = np.linspace(0, 8, 81)
    frames = [go.Frame(name=f"{t:.1f}", data=dynamic_traces(pair, t),
                       traces=list(range(offset, len(traces)))) for t in times]
    figure = go.Figure(data=traces, frames=frames)
    figure.update_layout(
        title="S-slalom 3D — Joint-risk vs Original CC-MPC (recorded seed 27)",
        template="plotly_white", height=850,
        scene=dict(xaxis_title="x (m)", yaxis_title="y (m)", zaxis_title="z (m)",
                   xaxis=dict(range=[-0.3, 3.4]), yaxis=dict(range=[-1.6, 1.6]),
                   zaxis=dict(range=[0.5, 1.9]), aspectmode="data",
                   camera=dict(eye=dict(x=1.6, y=-1.8, z=1.2))),
        uirevision="keep-camera",
        annotations=[dict(text="Drag: rotate • Scroll: zoom • Red: physical obstacle • Purple sphere: Joint-risk k=1 • Blue sphere: Original CC-MPC k=1. Not a safety proof.",
                          x=0, y=-0.05, xref="paper", yref="paper", showarrow=False)],
        sliders=[dict(currentvalue=dict(prefix="Simulation time (s): "), steps=[
            dict(label=f"{t:.1f}", method="animate", args=[[f"{t:.1f}"],
                 dict(mode="immediate", frame=dict(duration=0, redraw=True), transition=dict(duration=0))])
            for t in times])],
        updatemenus=[dict(type="buttons", buttons=[
            dict(label="Play", method="animate", args=[None, dict(fromcurrent=True,
                 frame=dict(duration=100, redraw=True), transition=dict(duration=0))]),
            dict(label="Pause", method="animate", args=[[None], dict(mode="immediate",
                 frame=dict(duration=0, redraw=False))])])],
    )
    output = ROOT / "outputs/five_obstacle_through/trajectory_3d.html"
    figure.write_html(output, include_plotlyjs=True, auto_play=False)
    print(output)


def export_dashboard(pair):
    config = pair["uniform"]["config"]
    data = {"names": [o["name"] for o in config.obstacles], "variants": {}}
    for name, value in pair.items():
        recording = value["recording"]
        times = value["time"]
        data["variants"][name] = dict(
            time=times.tolist(), states=value["states"][:, :3].tolist(),
            means=recording["estimated_obstacle_predictions"][:, :, 1, :].tolist(),
            radii=recording["tightened_safety_radius_horizons"][:, 1, :].tolist(),
            risk=value["delta"][:, 1:, :].sum(axis=1).tolist(),
            physical=[[obstacle_position(o, float(t)).tolist() for o in config.obstacles] for t in times],
        )
    data["physicalRadii"] = [o["radius"] for o in config.obstacles]
    page = '''<!doctype html><html lang="vi"><meta charset="utf-8">
<title>Quỹ đạo 3D và phân bổ rủi ro</title>
<style>
:root{--ink:#18243b;--muted:#66738a;--line:#dfe5ef;--card:#fff;--blue:#258df2;--purple:#b44de0}
*{box-sizing:border-box}body{margin:0;background:#f3f5fa;color:var(--ink);font:14px/1.45 system-ui,-apple-system,Segoe UI,sans-serif}
header{padding:16px 22px 12px;background:linear-gradient(135deg,#fff,#f8fbff);border-bottom:1px solid var(--line);box-shadow:0 2px 10px #18243b12}
.brand{display:flex;align-items:flex-start;justify-content:space-between;gap:18px}.brand h2{margin:0;font-size:21px;letter-spacing:-.02em}.subtitle{color:var(--muted);margin-top:2px}.mode-badge{padding:5px 10px;border-radius:999px;background:#eaf2ff;color:#1764b0;font-weight:700;white-space:nowrap}
#controls{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-top:12px;padding:10px;background:#f8fafc;border:1px solid var(--line);border-radius:12px}label{font-weight:700;color:#40516b}select,button{padding:8px 11px;border:1px solid #cbd5e1;border-radius:8px;background:white;color:var(--ink);font:inherit;cursor:pointer}button{background:#18243b;color:white;border-color:#18243b;font-weight:700}button:hover{background:#2a3b59}select:focus,button:focus,input:focus{outline:2px solid #8ac1ff;outline-offset:1px}#slider{flex:1;min-width:220px;accent-color:#356fd8}.time{min-width:70px;text-align:right;font-variant-numeric:tabular-nums}.mode-help{color:var(--muted);flex-basis:100%}#total{margin-top:8px;color:#40516b;font-variant-numeric:tabular-nums}
main{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(330px,.95fr);gap:12px;padding:12px 18px 0}.panel{background:var(--card);border:1px solid var(--line);border-radius:14px;overflow:hidden;box-shadow:0 2px 8px #18243b0a}.right-stack{display:grid;grid-template-rows:270px 270px;gap:12px}#scene{height:600px}#bars,#history{height:270px}.note{margin:12px 18px;padding:10px 14px;background:#fff;border:1px solid var(--line);border-radius:10px;color:#53627a;font-size:12px}
@media(max-width:900px){main{display:block;padding:10px}.right-stack{margin-top:10px;grid-template-rows:260px 260px}#scene{height:520px}.note{margin:10px}}
</style>
<header><div class="brand"><div><h2>Quỹ đạo 3D · Phân bổ ngân sách rủi ro</h2><div class="subtitle">Five-obstacle S-slalom · seed 27 · recording replay</div></div><span class="mode-badge" id="modeBadge">CC-MPC gốc</span></div><div id="controls">
<label for="mode">Chế độ xem</label><select id="mode"><option value="uniform">Trường hợp 1: CC-MPC gốc</option><option value="joint_hard">Trường hợp 2: CC-MPC + Joint-risk</option></select>
<button id="play">▶ Chạy</button><input id="slider" type="range" min="0" max="159" step="1" value="0"><strong class="time" id="clock">0.00 s</strong><div class="mode-help" id="modeHelp"></div></div>
<div id="total"></div></header><main><div class="panel" id="scene"></div><section class="right-stack"><div class="panel" id="bars"></div><div class="panel" id="history"></div></section></main>
<div class="note">Kéo chuột để xoay, cuộn để zoom. Đỏ: vật cản thật. Cầu mờ: tâm ước lượng và bán kính ràng buộc k=1 (CC-MPC: xanh; Joint-risk: tím).<br>
Cột δᵢ = tổng phân bổ rủi ro cho vật cản i trên các bước k=1…N của horizon hiện tại; tổng ngân sách 0.10. Đây không phải xác suất va chạm thực nghiệm. Một seed minh họa, không phải chứng minh an toàn hay real-time 20 Hz.</div>
<script>__PLOTLY__</script><script>const D=__DATA__;
const palette=['#0072b2','#e69f00','#009e73','#cc79a7','#d55e00'];
const mode=document.getElementById('mode'), slider=document.getElementById('slider'), play=document.getElementById('play');
let timer=null,busy=false,pending=false,renderVersion=0; const opts={responsive:true,displaylogo:false};
function ball(c,r,color,opacity,name){let x=[],y=[],z=[];for(let a=0;a<=12;a++){let xx=[],yy=[],zz=[];let v=a*Math.PI/12;for(let b=0;b<=20;b++){let u=b*2*Math.PI/20;xx.push(c[0]+r*Math.cos(u)*Math.sin(v));yy.push(c[1]+r*Math.sin(u)*Math.sin(v));zz.push(c[2]+r*Math.cos(v));}x.push(xx);y.push(yy);z.push(zz);}return {type:'surface',x,y,z,colorscale:[[0,color],[1,color]],opacity,showscale:false,showlegend:false,name,hovertemplate:name+'<extra></extra>'};}
async function draw(){if(busy){pending=true;return;}busy=true;try{do{pending=false;const version=renderVersion;
let key=mode.value,v=D.variants[key],i=+slider.value,t=v.time[i],col=key==='uniform'?'#258df2':'#b44de0';
document.getElementById('modeBadge').textContent=key==='uniform'?'CC-MPC gốc':'CC-MPC + Joint-risk';
document.getElementById('modeHelp').textContent=key==='uniform'?'Trường hợp 1 · phân bổ uniform, không tối ưu joint-risk.':'Trường hợp 2 · bật phân bổ joint-risk required.';
document.getElementById('clock').textContent=t.toFixed(2)+' s';
let total=v.risk[i].reduce((a,b)=>a+b,0);document.getElementById('total').textContent='Tổng δ = '+total.toFixed(6)+' / 0.100000 · Bán kính lấy trực tiếp từ recording';
let xyz=k=>v.states.map(p=>p[k]);let traces=[{type:'scatter3d',x:xyz(0),y:xyz(1),z:xyz(2),mode:'lines',line:{color:col,width:6},name:'Quỹ đạo ghi lại'},
{type:'scatter3d',x:[v.states[i][0]],y:[v.states[i][1]],z:[v.states[i][2]],mode:'markers',marker:{size:7,color:col},name:'Drone'},
{type:'scatter3d',x:[0,3],y:[0,0],z:[1.2,1.2],mode:'markers+text',text:['Start','Goal'],marker:{size:6,color:['black','#d6a000']},name:'Start / Goal'}];
D.names.forEach((n,j)=>{traces.push(ball(v.physical[i][j],D.physicalRadii[j],'#db4c43',0.9,n));traces.push(ball(v.means[i][j],v.radii[i][j],col,0.13,n+' · k=1 · R='+v.radii[i][j].toFixed(5)+' m'));});
await Plotly.react('scene',traces,{title:mode.options[mode.selectedIndex].text,uirevision:'camera',margin:{l:0,r:0,b:0,t:45},scene:{aspectmode:'data',xaxis:{title:'x (m)',range:[-0.3,3.4]},yaxis:{title:'y (m)',range:[-1.6,1.6]},zaxis:{title:'z (m)',range:[0.5,1.9]}}},opts);
await Plotly.react('bars',[{type:'bar',x:D.names,y:v.risk[i],marker:{color:palette},text:v.risk[i].map(x=>x.toFixed(5)),textposition:'auto',hovertemplate:'%{x}<br>δᵢ=%{y:.6f}<extra></extra>'}],{title:'Phân bổ rủi ro từng vật cản tại '+t.toFixed(2)+' s',margin:{t:50,b:80},yaxis:{title:'δᵢ = Σₖ δₖᵢ',range:[0,0.105]}},opts);
let history=D.names.map((n,j)=>({type:'scatter',mode:'lines',x:v.time,y:v.risk.map(r=>r[j]),name:n,line:{color:palette[j],width:2}}));
await Plotly.react('history',history,{title:'Phân bổ theo thời gian · vạch đứng = thời điểm đang xem',margin:{t:50,b:45},xaxis:{title:'Thời gian (s)',range:[0,8]},yaxis:{title:'δᵢ',range:[0,0.105]},legend:{orientation:'h'},shapes:[{type:'line',xref:'x',yref:'paper',x0:t,x1:t,y0:0,y1:1,line:{color:'#222',dash:'dot'}}]},opts);
if(renderVersion!==version)pending=true;}while(pending);}finally{busy=false;if(pending)draw();}}
function stop(){if(timer){clearInterval(timer);timer=null;}play.textContent='▶ Chạy';}
function resetMode(){stop();slider.value=0;renderVersion++;pending=true;draw();}
slider.oninput=()=>{renderVersion++;draw();};mode.onchange=resetMode;
play.onclick=function(){if(timer){stop();return;}this.textContent='⏸ Dừng';timer=setInterval(()=>{if(+slider.value>=159){stop();return;}slider.value=+slider.value+1;renderVersion++;draw();},100);};draw();
</script></html>'''
    page = page.replace('__PLOTLY__', get_plotlyjs()).replace('__DATA__', json.dumps(data))
    output = ROOT / 'outputs/five_obstacle_through/trajectory_3d.html'
    output.write_text(page, encoding='utf-8')
    print(output)


if __name__ == "__main__":
    main()
