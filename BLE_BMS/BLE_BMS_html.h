// BLE_BMS_html.h
// HTML page constants live in this header so that the Arduino IDE preprocessor
// (which only scans .ino files for auto-generated prototypes) never sees the
// raw-string content.  Without this split, the ctags-based scanner in Arduino
// IDE 1.x mis-identifies JavaScript "function" declarations inside the raw
// strings as C++ function definitions, generates broken prototypes, and causes
// "'function' does not name a type" compile errors.

#pragma once
#include <pgmspace.h>

static const char ROOT_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BMS Multi-Battery Monitor</title>
<style>
body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}
h1{color:#e0c97f;margin-bottom:4px}h2{color:#a0b4cc;margin-top:20px;margin-bottom:6px}
table{width:100%;border-collapse:collapse}th,td{padding:8px 10px;text-align:center;border-bottom:1px solid #2a2a4a}
th{background:#16213e;color:#a0b4cc}.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:10px;margin-bottom:16px}
.card{background:#16213e;border-radius:8px;padding:12px;text-align:center}.card .label{font-size:.75em;color:#8899aa;margin-bottom:4px}
.card .value{font-size:1.4em;font-weight:bold}.green{color:#4caf50}.amber{color:#ff9800}.red{color:#f44336}.mono{font-family:monospace}
.btn-mgr{display:inline-block;padding:7px 18px;background:#2a6ea6;color:#fff;border-radius:5px;text-decoration:none;font-size:.95em;margin-bottom:12px}
.note{color:#8899aa;font-size:.9em}
</style>
</head>
<body>
<h1>Battery BMS (Persistent BLE)</h1>
<p><a class="btn-mgr" href="/batteries">&#9881; Manage Batteries</a></p>
<div class="grid" id="cards"></div>
<h2>Per-battery status</h2>
<table>
<thead><tr><th>Name</th><th>MAC</th><th>Enabled</th><th>Connected</th><th>Voltage (V)</th><th>Current (A)</th><th>SoC (%)</th><th>Temp (C)</th><th>Min Cell (V)</th><th>Min Cell ID</th><th>Max Cell (V)</th><th>Max Cell ID</th><th>Data age (s)</th><th>Drops</th></tr></thead>
<tbody id="batteryRows"><tr><td colspan="14">Loading&#8230;</td></tr></tbody>
</table>
<p class="note" id="summaryStatus">Loading summary&#8230;</p>
<script>
const cardsEl = document.getElementById('cards');
const rowsEl = document.getElementById('batteryRows');
const statusEl = document.getElementById('summaryStatus');
function fmt(v, dp){dp=dp||0;return (v==null||isNaN(+v))?'--':Number(v).toFixed(dp);}
function esc(v){return String(v==null?'':v).replace(/[&<>"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});}
function card(label,value,cls){cls=cls||'';return '<div class="card"><div class="label">'+esc(label)+'</div><div class="value '+cls+'">'+esc(value)+'</div></div>';}
function renderSummary(data){
  var a=data.aggregate||{};
  var cards=[
    card('Enabled Batteries',data.enabledCount!=null?data.enabledCount:'--'),
    card('Connected Batteries',data.connectedCount!=null?data.connectedCount:'--')
  ];
  if(a.valid){
    cards.push(card('Aggregate Voltage',fmt(a.voltage,2)+' V'));
    cards.push(card('Aggregate Current',fmt(a.current,2)+' A'));
    cards.push(card('Aggregate SoC',(a.soc!=null?a.soc:'--')+' %'));
    cards.push(card('Aggregate Temp',a.hasTemperature?fmt(a.temperature,1)+' C':'n/a'));
    cards.push(card('Fresh Batteries',a.contributingBatteries!=null?a.contributingBatteries:0));
    cards.push(card('Last Fresh Data',a.ageSec>=0?a.ageSec+' s ago':'n/a'));
  }else{
    cards.push(card('Aggregate','No fresh data','amber'));
  }
  cardsEl.innerHTML=cards.join('');
  var rows=(data.batteries||[]).map(function(b){
    return '<tr>'
      +'<td><a href="/battery?index='+b.index+'">'+esc(b.name)+'</a></td>'
      +'<td class="mono">'+esc(b.mac)+'</td>'
      +'<td>'+(b.enabled?'yes':'no')+'</td>'
      +'<td class="'+(b.connected?'green':'red')+'">'+(b.connected?'yes':'no')+'</td>'
      +'<td>'+(b.hasData?fmt(b.voltage,2):'-')+'</td>'
      +'<td>'+(b.hasData?fmt(b.current,2):'-')+'</td>'
      +'<td>'+(b.hasData?b.soc:'-')+'</td>'
      +'<td>'+(b.hasTemperature?fmt(b.temperature,1):'-')+'</td>'
      +'<td>'+(b.hasCellStats?fmt(b.minCellV,3):'-')+'</td>'
      +'<td>'+(b.hasCellStats?b.minCellIndex:'-')+'</td>'
      +'<td>'+(b.hasCellStats?fmt(b.maxCellV,3):'-')+'</td>'
      +'<td>'+(b.hasCellStats?b.maxCellIndex:'-')+'</td>'
      +'<td>'+(b.dataAgeSec>=0?b.dataAgeSec:'-')+'</td>'
      +'<td>'+b.disconnectCount+'</td>'
      +'</tr>';
  }).join('');
  rowsEl.innerHTML=rows||'<tr><td colspan="14">No configured batteries.</td></tr>';
  statusEl.textContent='WiFi '+data.wifi+', last refresh '+(new Date().toLocaleTimeString());
}
var refreshPending=false;
function refreshSummary(){
  if(refreshPending) return;
  refreshPending=true;
  fetch('/api/summary',{cache:'no-store'})
    .then(function(r){if(!r.ok)throw new Error('HTTP '+r.status);return r.json();})
    .then(function(data){refreshPending=false;renderSummary(data);})
    .catch(function(err){
      refreshPending=false;
      console.warn('summary fetch failed',err);
      statusEl.textContent='Summary refresh failed: '+err.message;
    });
}
refreshSummary();
setInterval(refreshSummary,4000);
</script>
</body>
</html>
)HTML";

static const char BATTERY_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Battery Cell Detail</title>
<style>
body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}
h1{color:#e0c97f;margin-bottom:4px}a{color:#8ec5ff}table{width:100%;border-collapse:collapse;margin-top:16px}
th,td{padding:8px 10px;text-align:center;border-bottom:1px solid #2a2a4a}
th{background:#16213e;color:#a0b4cc}.card{background:#16213e;border-radius:8px;padding:12px;margin:12px 0}.mono{font-family:monospace}.muted{color:#a0b4cc}
</style>
</head>
<body>
<p><a href="/">Back to summary</a></p>
<h1 id="title">Battery detail</h1>
<div class="card">
  <div>MAC: <span class="mono" id="mac">--</span></div>
  <div>Connected: <span id="connected">--</span></div>
  <div>Voltage / Current / SoC: <span id="basic">--</span></div>
  <div>Cell data age: <span id="age">n/a</span></div>
</div>
<div id="cellsBlock"><p class="muted">Loading cell data&#8230;</p></div>
<script>
var params=new URLSearchParams(location.search);
var index=params.get('index');
var titleEl=document.getElementById('title');
var macEl=document.getElementById('mac');
var connEl=document.getElementById('connected');
var basicEl=document.getElementById('basic');
var ageEl=document.getElementById('age');
var cellsBlock=document.getElementById('cellsBlock');
function fmt(v,dp){dp=dp||0;return (v==null||isNaN(+v))?'--':Number(v).toFixed(dp);}
function esc(v){return String(v==null?'':v).replace(/[&<>"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});}
function renderBattery(data){
  titleEl.textContent='Battery detail: '+data.name;
  macEl.textContent=data.mac;
  connEl.textContent=data.connected?'yes':'no';
  basicEl.textContent=data.hasData?fmt(data.voltage,2)+' V / '+fmt(data.current,2)+' A / '+data.soc+'%':'No recent battery data';
  ageEl.textContent=data.cellDataAgeSec>=0?data.cellDataAgeSec+' s':'n/a';
  if(!data.hasCellData||!(data.cells||[]).length){
    cellsBlock.innerHTML='<p class="muted">No valid cell data available yet.</p>';
    return;
  }
  var rows=data.cells.map(function(v,idx){return '<tr><td>'+(idx+1)+'</td><td>'+fmt(v,3)+'</td></tr>';}).join('');
  cellsBlock.innerHTML='<h2>Cell voltages ('+data.cells.length+')</h2><table><tr><th>Cell</th><th>Voltage (V)</th></tr>'+rows+'</table>';
}
function refreshBattery(){
  if(index==null){cellsBlock.innerHTML='<p class="muted">Missing battery index.</p>';return;}
  fetch('/api/battery?index='+encodeURIComponent(index),{cache:'no-store'})
    .then(function(r){if(!r.ok)throw new Error('HTTP '+r.status);return r.json();})
    .then(renderBattery)
    .catch(function(err){
      cellsBlock.innerHTML='<p class="muted">Failed to load battery detail: '+esc(err.message)+'</p>';
    });
}
refreshBattery();
setInterval(refreshBattery,3000);
</script>
</body>
</html>
)HTML";

static const char BATTERIES_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Manage Batteries</title>
<style>
body{font-family:sans-serif;margin:16px;background:#1a1a2e;color:#eee}
h1{color:#e0c97f;margin-bottom:6px}h2{color:#a0b4cc;margin-top:20px;margin-bottom:6px;font-size:1.05em}
table{width:100%;border-collapse:collapse;margin-bottom:12px}th,td{padding:8px 10px;text-align:left;border-bottom:1px solid #2a2a4a}
th{background:#16213e;color:#a0b4cc}.mono{font-family:monospace;font-size:.9em}
.btn{display:inline-block;padding:6px 16px;border-radius:4px;border:none;cursor:pointer;font-size:.95em;color:#fff;text-decoration:none}
.btn-back{background:#37474f}.btn-scan{background:#2a6ea6}.btn-add{background:#2e7d32;font-weight:bold;padding:8px 22px;font-size:1em}.btn-remove{background:#c62828}
.note{color:#8899aa;font-size:.9em;margin:4px 0}.ok{color:#4caf50}.bad{color:#f44336}.warn{color:#ff9800}
input[type=radio]{accent-color:#2a6ea6;width:18px;height:18px;cursor:pointer}input[type=text]{background:#16213e;color:#eee;border:1px solid #444;border-radius:3px;padding:3px 6px;width:140px}
button:disabled{opacity:.6;cursor:not-allowed}
</style>
</head>
<body>
<h1>&#9889; Manage Batteries</h1>
<p><a class="btn btn-back" href="/">&#8592; Back to Summary</a></p>
<p class="note" id="pageStatus">Loading batteries&#8230;</p>
<h2 id="activeTitle">Active batteries</h2>
<div id="activeBlock"><p class="note">Loading&#8230;</p></div>
<h2>Discover new batteries</h2>
<p><button class="btn btn-scan" id="scanBtn" type="button">&#128269; Scan for BMS Devices</button></p>
<p class="note">Scan runs in the background. All nearby BLE devices are listed as candidates &mdash; choose your BMS device by MAC address or name.</p>
<div id="scanInfo" class="note">Scan idle.</div>
<h2>Candidates</h2>
<div id="candidateBlock"><p class="note">No scan results yet.</p></div>
<p><button class="btn btn-add" id="addBtn" type="button">&#43; ADD TO ACTIVE LIST</button></p>
<script>
var batteriesState=null;
var selectedCandidate=null;
var candidateNames={};
var pageStatus=document.getElementById('pageStatus');
var activeTitle=document.getElementById('activeTitle');
var activeBlock=document.getElementById('activeBlock');
var scanInfo=document.getElementById('scanInfo');
var candidateBlock=document.getElementById('candidateBlock');
var scanBtn=document.getElementById('scanBtn');
var addBtn=document.getElementById('addBtn');
function esc(v){return String(v==null?'':v).replace(/[&<>"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});}
function candidateDefaultName(c){
  var key=String(c.index);
  if(!(key in candidateNames)) candidateNames[key]=c.name||c.mac;
  return candidateNames[key];
}
function updateCandidateName(index,value){candidateNames[String(index)]=value;}
function renderActive(configured){
  activeTitle.textContent='Active batteries ('+configured.length+'/'+batteriesState.maxBatteries+')';
  if(!configured.length){
    activeBlock.innerHTML='<p class="note">No batteries configured. Use Scan below to discover and add batteries.</p>';
    return;
  }
  var rows=configured.map(function(b){
    return '<tr>'
      +'<td>'+esc(b.name)+'</td>'
      +'<td class="mono">'+esc(b.mac)+'</td>'
      +'<td class="'+(b.connected?'ok':'bad')+'">'+(b.connected?'yes':'no')+'</td>'
      +'<td><button class="btn btn-remove remove-btn" type="button" data-index="'+b.index+'" data-name="'+esc(b.name)+'">Remove</button></td>'
      +'</tr>';
  }).join('');
  activeBlock.innerHTML='<table><tr><th>Name</th><th>MAC</th><th>Connected</th><th>Remove</th></tr>'+rows+'</table>';
  activeBlock.querySelectorAll('.remove-btn').forEach(function(btn){
    btn.addEventListener('click',function(){removeBattery(Number(btn.dataset.index),btn.dataset.name||'battery');});
  });
}
function renderCandidates(candidates){
  if(!candidates.length){
    candidateBlock.innerHTML='<p class="note">No candidates available. Start a scan.</p>';
    addBtn.disabled=true;
    selectedCandidate=null;
    return;
  }
  if(!candidates.some(function(c){return c.index===selectedCandidate;})) selectedCandidate=candidates[0].index;
  var rows=candidates.map(function(c){
    return '<tr>'
      +'<td><input type="radio" name="candidate" '+(c.index===selectedCandidate?'checked':'')+' onchange="selectedCandidate='+c.index+'"></td>'
      +'<td class="mono">'+esc(c.mac)+'</td>'
      +'<td>'+esc(c.name)+'</td>'
      +'<td><input id="cand-name-'+c.index+'" type="text" maxlength="19" value="'+esc(candidateDefaultName(c))+'" oninput="updateCandidateName('+c.index+',this.value)"></td>'
      +'</tr>';
  }).join('');
  candidateBlock.innerHTML='<table><tr><th>Select</th><th>MAC</th><th>Advertised Name</th><th>Display Name</th></tr>'+rows+'</table>';
  addBtn.disabled=false;
}
function renderState(data){
  batteriesState=data;
  var pending=data.action&&data.action.pending;
  var actionCls=data.action&&data.action.ok?'ok':'warn';
  pageStatus.className='note '+actionCls;
  pageStatus.textContent=data.action?data.action.message:'Ready';
  renderActive(data.configured||[]);
  renderCandidates(data.candidates||[]);
  var scan=data.scan||{};
  if(scan.inProgress){
    scanInfo.className='note warn';
    scanInfo.textContent='Background scan running\u2026 '+scan.resultCount+' candidate(s) found so far.';
  }else if(scan.done){
    scanInfo.className='note';
    scanInfo.textContent=scan.resultCount?'Scan complete: '+scan.resultCount+' candidate(s) available.':'Scan complete: no new supported batteries found.';
  }else if(scan.requested){
    scanInfo.className='note warn';
    scanInfo.textContent='Scan queued\u2026';
  }else{
    scanInfo.className='note';
    scanInfo.textContent='Scan idle.';
  }
  scanBtn.disabled=!!(scan.requested||scan.inProgress||pending);
  addBtn.disabled=!((data.candidates||[]).length)||pending||data.batteryCount>=data.maxBatteries;
}
var refreshBatteriesPending=false;
function refreshBatteries(){
  if(refreshBatteriesPending) return;
  refreshBatteriesPending=true;
  fetch('/api/batteries',{cache:'no-store'})
    .then(function(r){if(!r.ok)throw new Error('HTTP '+r.status);return r.json();})
    .then(function(data){refreshBatteriesPending=false;renderState(data);})
    .catch(function(err){
      refreshBatteriesPending=false;
      console.warn('batteries fetch failed',err);
      pageStatus.className='note bad';
      pageStatus.textContent='Failed to refresh batteries: '+err.message;
    });
}
function callApi(url){
  return fetch(url,{cache:'no-store'})
    .then(function(r){
      return r.json().catch(function(){return {};}).then(function(body){
        if(!r.ok) throw new Error(body.error||'HTTP '+r.status);
        renderState(body);
      });
    });
}
function removeBattery(index,name){
  if(!confirm('Remove '+name+'?')) return;
  callApi('/api/batteries/remove?index='+encodeURIComponent(index))
    .catch(function(err){
      pageStatus.className='note bad';
      pageStatus.textContent='Remove failed: '+err.message;
    });
}
scanBtn.addEventListener('click',function(){
  callApi('/api/batteries/scan')
    .catch(function(err){
      pageStatus.className='note bad';
      pageStatus.textContent='Scan request failed: '+err.message;
    });
});
addBtn.addEventListener('click',function(){
  if(selectedCandidate==null) return;
  var input=document.getElementById('cand-name-'+selectedCandidate);
  var chosenName=(input&&input.value.trim())||'';
  callApi('/api/batteries/add?idx='+encodeURIComponent(selectedCandidate)+'&name='+encodeURIComponent(chosenName))
    .catch(function(err){
      pageStatus.className='note bad';
      pageStatus.textContent='Add failed: '+err.message;
    });
});
refreshBatteries();
setInterval(refreshBatteries,3000);
</script>
</body>
</html>
)HTML";
