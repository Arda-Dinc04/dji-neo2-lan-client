"""Local programmatic client for the same one-shot HTTP operator protocol.

This client runs on the PC owning the controller, so both ends use the same
monotonic clock. WAN browser control uses the HTML client and PC-issued windows.
An uncertain mutation is never retried and no background lease renewal exists.
"""
import json
from dataclasses import dataclass,field
import math
import re
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlsplit


class OperatorError(RuntimeError):
    pass


@dataclass(frozen=True)
class PreviewFrame:
    data: bytes=field(repr=False)
    connection_generation: str
    frame_id: str
    encoded_age_seconds: float
    read_started_at: float
    read_finished_at: float


class FlightOperatorClient:
    MAX_RESPONSE_BYTES=256*1024
    MAX_FRAME_BYTES=4*1024*1024

    def __init__(self,base_url='http://127.0.0.1:8794',*,timeout=2.):
        parsed=urlsplit(base_url)
        if (parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost')
                or parsed.username or parsed.password or parsed.path not in ('','/')
                or parsed.query or parsed.fragment):raise ValueError('operator_client_requires_loopback')
        self.base_url=base_url.rstrip('/')
        if type(timeout) not in (int,float) or not math.isfinite(timeout) or not 0<timeout<=5:
            raise ValueError('operator_timeout_invalid')
        self.timeout=timeout
        self._mutation=threading.Lock()
        self._expected_context=None
        # Disable proxies and redirects: the private page token stays on this
        # exact loopback origin even when the user's environment has a proxy.
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self,*args):return None
        self._opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect())
        page=self._read('/')
        match=re.search(rb'<meta name="neo2-token" content="([0-9a-f]{64})"',page)
        if match is None:raise OperatorError('operator_page_token_unavailable')
        self._token=match.group(1).decode('ascii')

    def _read(self,path,payload=None):
        return self._read_response(path,payload,maximum_bytes=self.MAX_RESPONSE_BYTES)[0]

    def _read_response(self,path,payload=None,*,maximum_bytes):
        headers={}
        if hasattr(self,'_token'):headers['X-Neo2-Token']=self._token
        if payload is not None:headers['Content-Type']='application/json'
        request=urllib.request.Request(self.base_url+path,headers=headers,
            data=None if payload is None else json.dumps(payload,allow_nan=False).encode())
        try:
            with self._opener.open(request,timeout=self.timeout) as response:
                data=response.read(maximum_bytes+1)
                if len(data)>maximum_bytes:raise OperatorError('operator_response_too_large')
                return data,response.headers
        except urllib.error.HTTPError as error:
            try:value=json.loads(error.read(4096)).get('error')
            except (ValueError,AttributeError):value=None
            reason=value if isinstance(value,str) and re.fullmatch('[a-z][a-z0-9_]{0,95}',value) else 'operator_request_rejected'
            raise OperatorError(reason) from None
        except (OSError,urllib.error.URLError):raise OperatorError('operator_request_uncertain') from None

    def _json(self,path,payload=None):
        try:value=json.loads(self._read(path,payload))
        except ValueError:raise OperatorError('operator_response_invalid') from None
        if not isinstance(value,dict):raise OperatorError('operator_response_invalid')
        return value

    def status(self):
        return self._json('/api/flight/status')

    def read_frame(self):
        """Read a pinned local preview; its file age is not camera acquisition age.

        The caller checks owner identity in status before and after this GET.
        No operator window, connection request or actuator command is involved.
        """
        if not hasattr(self,'_token'):
            raise OperatorError('camera_frame_requires_local_client')
        if self._expected_context is None:
            raise OperatorError('camera_frame_requires_pinned_connection')
        started=time.monotonic()
        data,headers=self._read_response('/api/flight/frame',maximum_bytes=self.MAX_FRAME_BYTES)
        finished=time.monotonic()
        generation=headers.get('X-Neo2-Connection-Generation')
        identity=headers.get('X-Neo2-Frame-Id')
        try:age=float(headers.get('X-Neo2-Frame-Age-Seconds','nan'))
        except ValueError:age=float('nan')
        if generation!=self._expected_context[1]:
            raise OperatorError('camera_frame_generation_changed')
        if (headers.get_content_type()!='image/jpeg' or not data.startswith(b'\xff\xd8')
                or not data.endswith(b'\xff\xd9') or not math.isfinite(age) or age<0
                or not isinstance(identity,str) or not re.fullmatch(r'[0-9a-f]{1,32}(?::[0-9a-f]{1,32}){3}',identity)
                or int(identity.rsplit(':',1)[1],16)!=len(data)):
            raise OperatorError('camera_frame_metadata_invalid')
        return PreviewFrame(data,generation,identity,age,started,finished)

    def pin_connection(self,status):
        epoch=status.get('controller_owner',{}).get('epoch')
        generation=status.get('connection',{}).get('generation')
        if (not isinstance(epoch,str) or not re.fullmatch('[0-9a-f]{48}',epoch)
                or not isinstance(generation,str) or not re.fullmatch('[0-9a-f]{24}',generation)):
            raise OperatorError('operator_connection_identity_unavailable')
        expected=(epoch,generation)
        if self._expected_context is not None and self._expected_context!=expected:
            raise OperatorError('operator_request_generation_changed')
        self._expected_context=expected

    def _submit(self,endpoint,action,parameters):
        if not self._mutation.acquire(blocking=False):raise OperatorError('operator_mutation_busy')
        try:
            started=time.monotonic()
            window=self._json('/api/flight/window')
            if time.monotonic()-started>=.250:raise OperatorError('operator_window_expired_before_submit')
            if (not isinstance(window.get('ticket'),str)
                    or type(window.get('mutation_sequence')) is not int
                    or not 1<=window['mutation_sequence']<(1<<53)):
                raise OperatorError('operator_window_invalid')
            current=(window.get('owner_epoch'),window.get('connection_generation'))
            expected=self._expected_context if self._expected_context is not None else current
            if expected!=current:raise OperatorError('operator_request_generation_changed')
            return self._json('/api/flight/'+endpoint,{'action':action,'parameters':parameters,
                'ingress':{'ticket':window['ticket'],'mutation_sequence':window['mutation_sequence'],
                    'expected_owner_epoch':expected[0],'expected_connection_generation':expected[1]}})
        finally:self._mutation.release()

    def intent(self,action,parameters=None):
        return self._submit('intent',action,{} if parameters is None else parameters)

    def camera(self,action,parameters=None):
        return self._submit('camera',action,{} if parameters is None else parameters)
