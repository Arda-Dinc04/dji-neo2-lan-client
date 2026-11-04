"""Generate a self-contained interactive bit-level protocol report."""

from __future__ import annotations

import html
import json
from pathlib import Path

from .capture import load_capture
from .protocol import decode_datagram


def generate_report(capture_path: str | Path, output_path: str | Path) -> Path:
    events = _redecode(load_capture(capture_path))
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(events, ensure_ascii=False).replace("</", "<\\/")
    title = html.escape(Path(capture_path).name)
    document = _TEMPLATE.replace("__TITLE__", title).replace("__EVENTS__", data)
    output.write_text(document, encoding="utf-8", newline="\n")
    return output.resolve()


def _redecode(events: list[dict]) -> list[dict]:
    """Re-derive the annotations from the raw bytes of each captured event.

    The raw hex is the evidence; the stored annotations are only the reading
    that the decoder produced on the day of the capture. Reporting from the
    bytes makes every later decoder improvement apply to the whole archive.
    """

    for event in events:
        if event.get("interface") != "udp":
            continue
        try:
            raw = bytes.fromhex(event["raw_hex"])
        except (KeyError, TypeError, ValueError):
            continue
        event["decoded"] = decode_datagram(raw)
    return events


_TEMPLATE = r'''<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Neo 2 — __TITLE__</title>
<style>
:root{color-scheme:dark;--bg:#07100e;--panel:#0e1b17;--line:#284139;--text:#ecf8f2;--muted:#91a89e;--tx:#e7ae55;--rx:#5cd1a2}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 system-ui,sans-serif}header{padding:18px 22px;border-bottom:1px solid var(--line)}h1,h2,p{margin:0}h1{font-size:20px}header p{color:var(--muted);margin-top:4px}.layout{display:grid;grid-template-columns:minmax(280px,34%) 1fr;min-height:calc(100vh - 78px)}aside{border-right:1px solid var(--line);overflow:auto;max-height:calc(100vh - 78px)}button.packet{display:grid;grid-template-columns:48px 34px 1fr;width:100%;gap:8px;padding:10px 12px;border:0;border-bottom:1px solid var(--line);background:transparent;color:var(--text);text-align:left;cursor:pointer}.packet:hover,.packet[aria-pressed=true]{background:var(--panel)}.packet .seq{color:var(--muted)}.packet .dir{font-weight:700}.packet.tx .dir{color:var(--tx)}.packet.rx .dir{color:var(--rx)}.packet small{display:block;color:var(--muted)}main{padding:18px;overflow:auto}.summary{display:flex;gap:16px;flex-wrap:wrap;margin-bottom:14px}.summary span{color:var(--muted)}.summary strong{color:var(--text)}.bits{display:grid;grid-template-columns:repeat(8,minmax(28px,1fr));gap:3px;margin:10px 0 18px}.bit{min-height:42px;padding:4px 2px;border:1px solid var(--line);background:var(--panel);font:12px ui-monospace,monospace;text-align:center}.bit b{display:block;font-size:15px}.bit small{color:var(--muted)}.bit.marked{border-color:var(--field);background:color-mix(in srgb,var(--field) 18%,var(--panel))}.bit.f0{--field:#82aaff}.bit.f1{--field:#5cd1a2}.bit.f2{--field:#e7ae55}.bit.f3{--field:#d997ff}.bit.f4{--field:#ff7f78}.bit.f5{--field:#65c7e8}.hex{word-break:break-all;padding:12px;background:var(--panel);border:1px solid var(--line);font:13px/1.7 ui-monospace,monospace}.fields{width:100%;border-collapse:collapse;margin-top:12px}.fields th,.fields td{padding:8px;border-bottom:1px solid var(--line);text-align:left}.fields th{color:var(--muted)}.empty{color:var(--muted);padding:30px 0}@media(max-width:760px){.layout{grid-template-columns:1fr}aside{max-height:280px;border-right:0;border-bottom:1px solid var(--line)}main{padding:14px}.bits{grid-template-columns:repeat(8,minmax(24px,1fr))}}
</style>
</head>
<body>
<header><h1>DJI Neo 2 — inspection au bit près</h1><p id="capture-summary"></p></header>
<div class="layout"><aside id="packets" aria-label="Paquets capturés"></aside><main id="detail"><p class="empty">Sélectionner un paquet.</p></main></div>
<script>
const events=__EVENTS__;
const packets=document.getElementById('packets'),detail=document.getElementById('detail');
document.getElementById('capture-summary').textContent=`${events.length} événement(s) · ${events.filter(x=>x.direction==='tx').length} TX · ${events.filter(x=>x.direction==='rx').length} RX`;
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const hexValue=v=>typeof v==='number'?`0x${v.toString(16)}`:String(v);
function select(index){
 document.querySelectorAll('.packet').forEach((b,i)=>b.setAttribute('aria-pressed',i===index));
 const e=events[index],bytes=(e.raw_hex.match(/../g)||[]).map(x=>parseInt(x,16)),fields=e.decoded?.fields||[];
 const owners=[];fields.forEach((f,fi)=>{for(let n=f.offset_bits;n<f.offset_bits+f.width_bits;n++)owners[n]={name:f.name,index:fi}});
 let bits='';bytes.forEach((byte,bi)=>{for(let bit=7;bit>=0;bit--){const absolute=bi*8+bit,owner=owners[absolute];bits+=`<div class="bit ${owner?'marked f'+(owner.index%6):''}" title="${esc(owner?.name||'non identifié')}"><b>${(byte>>bit)&1}</b><small>${bi}.${bit}</small></div>`}});
 const rows=fields.map(f=>`<tr><td>${esc(f.name)}</td><td>${f.offset_bits}</td><td>${f.width_bits}</td><td><code>${esc(hexValue(f.value))}</code></td><td>${esc(f.description)}</td></tr>`).join('');
 detail.innerHTML=`<div class="summary"><span>Paquet <strong>#${e.sequence}</strong></span><span>Direction <strong>${esc(e.direction.toUpperCase())}</strong></span><span>Interface <strong>${esc(e.interface)}</strong></span><span>Taille <strong>${bytes.length} octets / ${bytes.length*8} bits</strong></span><span>Pair <strong>${esc(e.peer)}</strong></span></div><h2>Bits</h2><div class="bits" aria-label="Bits du paquet">${bits}</div><h2>Hexadécimal</h2><div class="hex">${bytes.map((x,i)=>`<span title="octet ${i}">${x.toString(16).padStart(2,'0')}</span>`).join(' ')}</div><h2>Champs décodés</h2><table class="fields"><thead><tr><th>Champ</th><th>Bit initial</th><th>Largeur</th><th>Valeur</th><th>Interprétation</th></tr></thead><tbody>${rows||'<tr><td colspan="5">Aucun champ identifié</td></tr>'}</tbody></table>`;
}
events.forEach((e,i)=>{const layers=(e.decoded?.layers||[]).map(x=>x.name).join(' → ')||'brut';const b=document.createElement('button');b.className=`packet ${e.direction}`;b.setAttribute('aria-pressed','false');b.innerHTML=`<span class="seq">#${e.sequence}</span><span class="dir">${esc(e.direction.toUpperCase())}</span><span>${esc(layers)}<small>${esc(e.timestamp)} · ${(e.raw_hex.length/2)} o</small></span>`;b.onclick=()=>select(i);packets.appendChild(b)});if(events.length)select(0);
</script>
</body>
</html>'''
