"""Loopback-only preview fixture with recorded delivery timing and no control routes."""
import argparse,bisect,hashlib,json,re,threading,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path

from neo2_control.reconstruction import _read
from neo2_control.room_mvs import write_new


def create_preview_replay(cache,*,port=0):
    cache=Path(cache)
    manifest=json.loads(_read(cache/'preview.json',8*1024**2));frames=manifest['frames']
    if (manifest.get('format')!='neo2-control/recorded-room-preview/v1'
            or not 1<=len(frames)<=4096 or manifest.get('poses_supplied') is not False):
        raise ValueError('room_preview_replay_manifest_invalid')
    first=frames[0]['recorded_available_at'];offsets=[];previous=-1.
    for row in frames:
        if not re.fullmatch(r'frame-[0-9]{5}\.jpg',row['file']):raise ValueError('room_preview_replay_filename_invalid')
        raw=_read(cache/'frames'/row['file'],4*1024**2)
        if len(raw)!=row['bytes'] or hashlib.sha256(raw).hexdigest()!=row['sha256']:
            raise ValueError('room_preview_replay_image_changed')
        offset=row['recorded_available_at']-first
        if not 0<=offset<=600 or offset<previous:raise ValueError('room_preview_replay_clock_invalid')
        offsets.append(offset);previous=offset
    state={'scope':'Offline JPEG fixture; no drone, connection or flight control',
        'source':manifest['source'],'origin_live_monotonic':None,'requests':[]}
    lock=threading.Lock()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def _record(self,method):
            row={'method':method,'path':self.path,'at':time.monotonic()}
            with lock:
                if len(state['requests'])>=50000:raise ValueError('room_preview_replay_request_limit')
                state['requests'].append(row)
            return row
        def do_POST(self):self._record('POST');self.send_error(405)
        def do_GET(self):
            entry=self._record('GET');headers={};kind='application/json'
            if self.path=='/':body=('<meta name="neo2-token" content="'+'a'*64+'">').encode();kind='text/html'
            elif self.path=='/api/flight/status':
                body=json.dumps({'connected':True,'controller_owner':{'epoch':'a'*48},
                    'connection':{'generation':'b'*24,'video':{'live':True}}}).encode()
            elif self.path=='/api/flight/frame':
                with lock:
                    now=time.monotonic()
                    if state['origin_live_monotonic'] is None:state['origin_live_monotonic']=now
                    origin=state['origin_live_monotonic']
                index=max(0,min(len(frames)-1,bisect.bisect_right(offsets,now-origin)-1))
                row=frames[index];body=_read(cache/'frames'/row['file'],4*1024**2)
                if hashlib.sha256(body).hexdigest()!=row['sha256']:self.send_error(409);return
                kind='image/jpeg';age=now-origin-offsets[index]
                entry.update(source_index=index,decoded_index=row['decoded_index'],sha256=row['sha256'],
                    available_at=origin+offsets[index],encoded_age_seconds=age)
                headers={'X-Neo2-Connection-Generation':'b'*24,
                    'X-Neo2-Frame-Id':f'{index:x}:a:b:{len(body):x}',
                    'X-Neo2-Frame-Age-Seconds':str(age)}
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
            for key,value in headers.items():self.send_header(key,value)
            self.end_headers();self.wfile.write(body)
    return ThreadingHTTPServer(('127.0.0.1',port),Handler),state


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cache',type=Path,required=True);p.add_argument('--audit',type=Path,required=True)
    p.add_argument('--port',type=int,default=0);args=p.parse_args()
    if args.audit.exists():raise ValueError('room_preview_replay_audit_must_be_new')
    server,state=create_preview_replay(args.cache,port=args.port)
    with server:
        print(json.dumps({'url':f'http://127.0.0.1:{server.server_port}',
            'scope':state['scope']}),flush=True)
        try:server.serve_forever()
        except KeyboardInterrupt:pass
        finally:
            args.audit.parent.mkdir(parents=True,exist_ok=True)
            write_new(args.audit,state)


if __name__=='__main__':main()
