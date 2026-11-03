"use strict";
(() => {
  const data = JSON.parse(document.getElementById("viewer-data").textContent);
  const $ = id => document.getElementById(id), canvas = $("map"), ctx = canvas.getContext("2d");
  const scenario = data.scenario, geometry = scenario.robot.geometry;
  const worlds = data.world, commands = data.commands;
  const wheel = [0,1].map(w => commands.filter(e => e[1] === "duty" && e[2] === w));
  const servos = commands.filter(e => e[1] === "servo");
  const rad = a => a*Math.PI/180;
  const stamp = e => Array.isArray(e) ? e[0] : e.t_us;
  function upper(items,t) { let a=0,b=items.length; while(a<b){const m=(a+b)>>1;if(stamp(items[m])<=t)a=m+1;else b=m;}return a; }
  const last = (items,t) => items[upper(items,t)-1] ?? null;
  const poseText = p => p ? `${p[0].toFixed(1)}, ${p[1].toFixed(1)} mm · ${p[2].toFixed(2)}°` : "unavailable";
  function point(p,x,y) {const c=Math.cos(rad(p[2])),s=Math.sin(rad(p[2]));return [p[0]+x*c-y*s,p[1]+x*s+y*c];}
  let time=0, playing=false, speed=1, width=1,height=1,dpr=1;
  let camera={x:0,y:0,scale:1}, view=null, drag=null;
  $("name").textContent=data.meta.name;
  $("source").textContent=`source ${data.meta.source_commit}`;
  $("scrub").max=String(data.meta.end_us);
  $("limits").textContent=data.omitted;
  for(const a of data.meta.assumptions){const li=document.createElement("li");li.textContent=`${a.path}: ${JSON.stringify(a.value)} — ${a.note}`;$("assumptions").append(li);}
  if(!data.meta.complete) $("warning").textContent=`INCOMPLETE TRACE — ${data.meta.error || "missing end record"}. Saved data ends at ${(data.meta.recorded_end_us/1e6).toFixed(6)} s.`;

  function state(t) {
    const n=upper(worlds,t)-1, sample=worlds[n] ?? null, next=worlds[n+1] ?? null;
    let pose=null, gap="";
    if(sample && t===sample[0]) pose=sample[1].slice();
    else if(sample && next && next[0]-sample[0]<=10000){
      const fraction=(t-sample[0])/(next[0]-sample[0]);
      pose=sample[1].map((v,i)=>v+(next[1][i]-v)*fraction);
    } else gap="World snapshot gap — true pose unavailable";
    const phase=last(data.phases,t), dt=last(data.dt,t);
    const beyond=t>data.meta.recorded_end_us;
    const phaseName=beyond ? "unavailable (past saved data)" : phase?.phase ?? "unavailable (no phase hooks)";
    return {t_us:t,sample,pose,gap,phase:phaseName,phaseEvidence:phase,
      dt,ghost:phaseName==="DETOUR" ? dt?.world_estimate ?? null : null,
      duties:wheel.map((events,i)=>last(events,t)?.[3] ?? sample?.[5]?.[i] ?? null),
      servo:last(servos,t) ? (last(servos,t)[2]-1500)*0.09 : sample?.[8] ?? null,
      trigger:last(data.triggers,t),result:last(data.results,t),
      console:data.console.slice(0,upper(data.console,t)).map(e=>e[1]).join("")};
  }
  const screen = p => [(p[0]-camera.x)*camera.scale+width/2,height/2-(p[1]-camera.y)*camera.scale];
  const worldAt = (x,y) => [camera.x+(x-width/2)/camera.scale,camera.y-(y-height/2)/camera.scale];
  function label(p,text,color="#244155",dx=0,dy=0){const [x,y]=screen(p);ctx.save();ctx.setTransform(dpr,0,0,dpr,0,0);ctx.font="12px system-ui";ctx.lineWidth=3;ctx.strokeStyle="#fffffff0";ctx.strokeText(text,x+dx,y+dy);ctx.fillStyle=color;ctx.fillText(text,x+dx,y+dy);ctx.restore();}
  function path(points,close=false){ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));if(close)ctx.closePath();}
  function outline(p,color,ghost=false){ctx.save();ctx.translate(p[0],p[1]);ctx.rotate(rad(p[2]));
    ctx.lineWidth=(ghost?2.5:2)/camera.scale;ctx.strokeStyle=color;ctx.fillStyle=ghost?"#ec9c2724":"#0079bb26";
    if(ghost)ctx.setLineDash([8/camera.scale,5/camera.scale]);
    path([[geometry.front_x_mm,geometry.width_mm/2],[geometry.rear_x_mm,geometry.width_mm/2],[geometry.rear_x_mm,-geometry.width_mm/2],[geometry.front_x_mm,-geometry.width_mm/2]],true);ctx.fill();ctx.stroke();
    ctx.setLineDash([]);path([[0,0],[geometry.front_x_mm+28,0],[geometry.front_x_mm+10,10],[geometry.front_x_mm+28,0],[geometry.front_x_mm+10,-10]]);ctx.stroke();ctx.beginPath();ctx.arc(0,0,4/camera.scale,0,2*Math.PI);ctx.fillStyle=color;ctx.fill();ctx.restore();
    label(p,ghost?"firmware estimate":"true pose",color,8,ghost?28:-18);
  }
  function floor(){const track=scenario.track,half=track.tile_size_mm/2;
    for(const tile of track.tiles){const x=track.origin_mm[0]+(tile.col+0.5)*track.tile_size_mm,y=track.origin_mm[1]+(tile.row+0.5)*track.tile_size_mm;
      ctx.save();ctx.translate(x,y);ctx.rotate(rad(tile.rotation_deg));ctx.fillStyle="#fffefa";ctx.fillRect(-half,-half,2*half,2*half);ctx.strokeStyle="#bcc8d0";ctx.lineWidth=1/camera.scale;ctx.strokeRect(-half,-half,2*half,2*half);
      ctx.beginPath();ctx.rect(-half,-half,2*half,2*half);ctx.clip();ctx.strokeStyle="#252b30";ctx.lineWidth=track.line_width_mm;ctx.lineCap="butt";
      if(tile.type==="straight" || tile.type==="cross"){path([[-half,0],[half,0]]);ctx.stroke();}
      if(tile.type==="cross"){path([[0,-half],[0,half]]);ctx.stroke();}
      if(tile.type==="curve"){ctx.beginPath();ctx.arc(-half,-half,track.curve_radius_mm,0,Math.PI/2);ctx.stroke();}ctx.restore();
    }
    for(const p of track.primitives){ctx.strokeStyle="#252b30";ctx.lineWidth=p.width_mm;ctx.lineCap="round";ctx.lineJoin="round";
      if(p.type==="polyline")path(p.points_mm,p.closed);else{ctx.beginPath();ctx.arc(...p.centre_mm,p.radius_mm,rad(p.start_deg),rad(p.start_deg+p.sweep_deg),p.sweep_deg<0);}ctx.stroke();}
    for(const obstacle of scenario.obstacles){ctx.save();ctx.translate(obstacle.pose.x_mm,obstacle.pose.y_mm);ctx.rotate(rad(obstacle.pose.heading_deg));ctx.fillStyle="#b95f44";ctx.strokeStyle="#803c2b";ctx.lineWidth=2/camera.scale;
      if(obstacle.type==="box"){ctx.beginPath();ctx.rect(-obstacle.size_mm[0]/2,-obstacle.size_mm[1]/2,...obstacle.size_mm);}else{ctx.beginPath();ctx.arc(0,0,obstacle.diameter_mm/2,0,2*Math.PI);}ctx.fill();ctx.stroke();ctx.restore();label([obstacle.pose.x_mm,obstacle.pose.y_mm],obstacle.id,"#7b3226",5,-7);}
  }
  function beam(){if(!view.pose || view.servo===null)return;
    const p=view.pose,axis=point(p,geometry.servo_axis_x_mm,0),h=p[2]+view.servo;
    const origin=point([axis[0],axis[1],h],geometry.sensor_face_offset_mm,0),length=511;
    ctx.save();ctx.translate(...origin);ctx.rotate(rad(h));ctx.beginPath();ctx.moveTo(0,0);ctx.arc(0,0,length,rad(-15),rad(15));ctx.closePath();ctx.fillStyle="#d99b1520";ctx.fill();ctx.strokeStyle="#ac841c80";ctx.lineWidth=1/camera.scale;ctx.stroke();
    path([[0,0],[length,0]]);ctx.strokeStyle="#ab7b00";ctx.stroke();ctx.restore();
  }
  function echo(){const e=view.trigger;
    if(e?.no_echo){const p=e.no_echo.point_mm,r=6/camera.scale;ctx.save();ctx.strokeStyle='#73798488';ctx.lineWidth=1.5/camera.scale;path([[p[0]-r,p[1]-r],[p[0]+r,p[1]+r]]);ctx.stroke();path([[p[0]-r,p[1]+r],[p[0]+r,p[1]-r]]);ctx.stroke();label(p,'sampled surface — no echo (incidence)','#737984',8,-10);ctx.restore();}
    if(!e?.hit || !e.point_mm || !e.origin_mm)return;
    const published=view.result && view.result.t_us>=e.t_us && view.result.ready_us===e.ready_us;
    const valid=published && ((view.result.word>>9)&3)===0;
    const color=valid?"#168044":"#a87e1b";
    ctx.strokeStyle=color;ctx.lineWidth=1.5/camera.scale;ctx.setLineDash([5/camera.scale,4/camera.scale]);path([e.origin_mm,e.point_mm]);ctx.stroke();ctx.setLineDash([]);
    ctx.beginPath();ctx.arc(...e.point_mm,5/camera.scale,0,2*Math.PI);ctx.stroke();if(valid){ctx.fillStyle=color;ctx.fill();}
    label(e.point_mm,`${published?"sampled surface":"pending surface"} ${e.distance_mm.toFixed(1)} mm${e.distance_mm>511?" (out of range)":""}`,color,8,-10);
  }
  function ir(){if(!view.pose || !view.sample)return;const raw=view.sample[6],cfg=scenario.robot.ir;
    cfg.points_mm.forEach((p,i)=>{const at=point(view.pose,...p),rise=Math.max(0,Math.min(1,(raw[i]-cfg.floor_adc[i])/(cfg.line_adc[i]-cfg.floor_adc[i])));
      ctx.beginPath();ctx.arc(...at,5/camera.scale,0,2*Math.PI);ctx.fillStyle=`hsl(145 70% ${24+rise*43}%)`;ctx.fill();ctx.strokeStyle="#fff";ctx.lineWidth=1/camera.scale;ctx.stroke();
      const [x,y]=screen(at),dy=(i-1)*27,dx=i===1?48:25;ctx.save();ctx.setTransform(dpr,0,0,dpr,0,0);ctx.strokeStyle="#32614c";ctx.lineWidth=1;path([[x,y],[x+dx-3,y+dy-4]]);ctx.stroke();ctx.restore();label(at,`${["L","C","R"][i]} ${raw[i]} · ${(100*rise).toFixed(0)}%`,"#175b39",dx,dy);
    });
  }
  function draw(){ctx.setTransform(1,0,0,1,0,0);ctx.clearRect(0,0,canvas.width,canvas.height);
    ctx.setTransform(camera.scale*dpr,0,0,-camera.scale*dpr,(width/2-camera.x*camera.scale)*dpr,(height/2+camera.y*camera.scale)*dpr);
    floor();ctx.strokeStyle="#2686b288";ctx.lineWidth=2/camera.scale;ctx.beginPath();let previous=null;
    for(let i=0;i<upper(worlds,time);i++){const row=worlds[i],p=row[1];if(previous && row[0]-previous[0]<=10000)ctx.lineTo(p[0],p[1]);else ctx.moveTo(p[0],p[1]);previous=row;}ctx.stroke();
    beam();echo();if(view.ghost)outline(view.ghost,"#b65a00",true);if(view.pose)outline(view.pose,"#0073b8");ir();
    for(const c of data.contacts){if(c.t_us>time)continue;const p=[c.x_mm,c.y_mm],size=8/camera.scale;ctx.strokeStyle="#c5253b";ctx.lineWidth=3/camera.scale;path([[p[0]-size,p[1]-size],[p[0]+size,p[1]+size]]);ctx.stroke();path([[p[0]-size,p[1]+size],[p[0]+size,p[1]-size]]);ctx.stroke();label(p,`contact pose ${c.obstacle_id??c.obstacle_index} @ ${(c.t_us/1e6).toFixed(3)}s`,"#b31d32",10,0);}
    ctx.save();ctx.setTransform(dpr,0,0,dpr,0,0);const magnitude=10**Math.floor(Math.log10(120/camera.scale)),length=[1,2,5,10].map(v=>v*magnitude).find(v=>v*camera.scale>=65);
    ctx.strokeStyle="#25485d";ctx.lineWidth=2;path([[18,25],[18+length*camera.scale,25]]);ctx.stroke();ctx.fillStyle="#25485d";ctx.font="12px system-ui";ctx.fillText(`${length} mm`,18,18);ctx.restore();
  }
  function render(){view=state(time);$("scrub").value=String(time);$("time").textContent=`${(time/1e6).toFixed(6)} s`;
    $("phase").textContent=view.phase+(view.phaseEvidence?" · inferred":"");
    $("phase").title=view.phaseEvidence?JSON.stringify(view.phaseEvidence):"No phase observation";
    $("step").textContent=view.dt?`${view.dt.step_name} @ ${(view.dt.t_us/1e6).toFixed(6)} s`:"unavailable";
    $("fail").textContent=view.dt?.fail_name??"unavailable";
    $("truth").textContent=poseText(view.pose)+(view.pose?" (drawing interpolation)":"");
    $("belief").textContent=view.ghost?poseText(view.ghost)+` · module ${view.dt.heading}°; held start-of-step`:"not available outside DETOUR";
    $("heading-gap").textContent=view.pose && view.ghost ? `${(view.pose[2]-view.ghost[2]).toFixed(2)}° (true − saved estimate)` : "unavailable";
    $("duties").textContent=view.duties.map(v=>v===null?"—":`${v}%`).join(" / ");
    $("counts").textContent=view.sample?.[4]?.join(" / ")??"unavailable";
    $("sample").textContent=view.sample?`${(view.sample[0]/1e6).toFixed(6)} s (raw/counts; not interpolated)`:"unavailable";
    const result=view.result,word=result?.word;
    $("ultra").textContent=result?`${["VALID","NO_RISE","STUCK_HIGH","NO_RETURN"][(word>>9)&3]}${((word>>9)&3)===0?((word&511)===0?" TOO_NEAR":` ${word&511} mm`):""} · streak ${(word>>11)&7} · word ${word} @ ${(result.t_us/1e6).toFixed(6)} s`:"no published result";
    $("console").textContent=view.console;$("console").scrollTop=$("console").scrollHeight;
    $("gap").textContent=view.gap;
    $("prev").disabled=!data.steps.some(e=>e.t_us<time);$("next").disabled=!data.steps.some(e=>e.t_us>time);
    draw();
  }
  function seek(us){time=Math.round(Math.max(0,Math.min(data.meta.end_us,us)));render();}
  function fit(){const points=[];const track=scenario.track,half=track.tile_size_mm/2;
    for(const tile of track.tiles){const x=track.origin_mm[0]+(tile.col+0.5)*track.tile_size_mm,y=track.origin_mm[1]+(tile.row+0.5)*track.tile_size_mm;points.push([x-half,y-half],[x+half,y+half]);}
    for(const primitive of track.primitives){if(primitive.type==="polyline")points.push(...primitive.points_mm);else points.push([primitive.centre_mm[0]-primitive.radius_mm,primitive.centre_mm[1]-primitive.radius_mm],[primitive.centre_mm[0]+primitive.radius_mm,primitive.centre_mm[1]+primitive.radius_mm]);}
    for(const o of scenario.obstacles){const radius=o.type==="box"?Math.hypot(...o.size_mm)/2:o.diameter_mm/2;points.push([o.pose.x_mm-radius,o.pose.y_mm-radius],[o.pose.x_mm+radius,o.pose.y_mm+radius]);}
    for(const row of worlds)points.push(row[1].slice(0,2));for(const row of data.dt)if(row.world_estimate)points.push(row.world_estimate.slice(0,2));
    if(!points.length){camera={x:0,y:0,scale:1};draw();return;}
    const bounds=points.reduce((b,p)=>[Math.min(b[0],p[0]),Math.max(b[1],p[0]),Math.min(b[2],p[1]),Math.max(b[3],p[1])],[Infinity,-Infinity,Infinity,-Infinity]);
    const loX=bounds[0]-200,hiX=bounds[1]+200,loY=bounds[2]-200,hiY=bounds[3]+200;
    camera={x:(loX+hiX)/2,y:(loY+hiY)/2,scale:Math.min(width/(hiX-loX),height/(hiY-loY))*.92};draw();
  }
  function focus(){if(!view.pose)return;const p=view.pose,g=view.ghost??p;camera={x:(p[0]+g[0])/2+100,y:(p[1]+g[1])/2,scale:Math.min(width/1200,height/1050)};draw();}
  function toggle(){playing=!playing;$("play").textContent=playing?"Pause":"Play";if(playing && time===data.meta.end_us)seek(0);}
  $("play").onclick=toggle;$("fit").onclick=fit;$("focus").onclick=focus;
  $("speed").onchange=e=>{speed=Number(e.target.value);};$("scrub").oninput=e=>seek(Number(e.target.value));
  $("prev").onclick=()=>{const prior=data.steps.filter(e=>e.t_us<time).at(-1);if(prior)seek(prior.t_us);};
  $("next").onclick=()=>{const next=data.steps.find(e=>e.t_us>time);if(next)seek(next.t_us);};
  document.addEventListener("keydown",e=>{if(e.code==="Space" && e.target.tagName!=="SELECT"){e.preventDefault();toggle();}});
  canvas.addEventListener("pointerdown",e=>{drag={x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId);});
  canvas.addEventListener("pointermove",e=>{if(!drag)return;camera.x-=(e.clientX-drag.x)/camera.scale;camera.y+=(e.clientY-drag.y)/camera.scale;drag={x:e.clientX,y:e.clientY};draw();});
  canvas.addEventListener("pointerup",()=>{drag=null;});canvas.addEventListener("pointercancel",()=>{drag=null;});
  canvas.addEventListener("wheel",e=>{e.preventDefault();const rect=canvas.getBoundingClientRect(),x=e.clientX-rect.left,y=e.clientY-rect.top,before=worldAt(x,y);
    camera.scale=Math.max(.02,Math.min(10,camera.scale*Math.exp(-e.deltaY*.0015)));camera.x=before[0]-(x-width/2)/camera.scale;camera.y=before[1]+(y-height/2)/camera.scale;draw();},{passive:false});
  let first=true;new ResizeObserver(()=>{const rect=canvas.getBoundingClientRect();width=rect.width;height=rect.height;dpr=window.devicePixelRatio||1;canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);if(first){first=false;fit();}render();}).observe(canvas);
  let priorFrame=performance.now();function frame(now){const delta=now-priorFrame;priorFrame=now;if(playing){seek(time+delta*1000*speed);if(time===data.meta.end_us){playing=false;$("play").textContent="Play";}}requestAnimationFrame(frame);}requestAnimationFrame(frame);
  window.simViewer=Object.freeze({seek,data,getState:()=>state(time),getCamera:()=>({...camera}),screen,fit,focus});
  render();
})();
