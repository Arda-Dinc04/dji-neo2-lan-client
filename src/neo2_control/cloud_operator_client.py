"""Authenticated HTTPS client; no PC token, redirects, proxies or POST retries."""
import json
from http.client import HTTPException
import math
import os
import re
import ssl
import stat
import urllib.error
import urllib.request
from urllib.parse import urlsplit

from .buffered_operator_client import BufferedFlightOperatorClient
from .operator_client import OperatorError


def _valid_bearer(value):
    return (isinstance(value,str) and 1<=len(value)<=8185
        and re.fullmatch(r'[A-Za-z0-9._~+/-]+=*',value) is not None)


def read_bearer_token(path):
    try:fd=os.open(path,os.O_RDONLY|os.O_CLOEXEC|os.O_NOFOLLOW|os.O_NONBLOCK)
    except OSError:raise ValueError('cloud_token_file_unavailable') from None
    with os.fdopen(fd,'rb') as stream:
        before=os.fstat(stream.fileno())
        if (not stat.S_ISREG(before.st_mode) or before.st_uid!=os.getuid()
                or stat.S_IMODE(before.st_mode)&0o077):
            raise ValueError('cloud_token_file_not_private')
        body=stream.read(8193);after=os.fstat(stream.fileno())
    if (len(body)>8192 or (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns)):
        raise ValueError('cloud_token_file_invalid')
    try:value=body.decode('ascii').strip()
    except UnicodeError:raise ValueError('cloud_bearer_token_invalid') from None
    if not _valid_bearer(value):raise ValueError('cloud_bearer_token_invalid')
    return value


class CloudFlightOperatorClient(BufferedFlightOperatorClient):
    PREFIX='/'

    def __init__(self,api_url,bearer_token,*,timeout=1.5,ssl_context=None):
        if (not isinstance(api_url,str) or len(api_url)>2048 or any(c in api_url for c in ('\\','?','#'))
                or any(ord(c)<=32 or ord(c)==127 for c in api_url)):
            raise ValueError('cloud_api_url_invalid')
        try:
            parsed=urlsplit(api_url)
            valid=(parsed.scheme=='https' and parsed.hostname and parsed.username is None and parsed.password is None
                and not parsed.query and not parsed.fragment and (parsed.port is None or 1<=parsed.port<=65535)
                and re.fullmatch(r'/(?:[A-Za-z0-9_-]+/)*[A-Za-z0-9_-]+/?',parsed.path))
        except ValueError:valid=False
        if not valid:raise ValueError('cloud_api_url_invalid')
        if not _valid_bearer(bearer_token):
            raise ValueError('cloud_bearer_token_invalid')
        if type(timeout) not in (int,float) or not math.isfinite(timeout) or not 0<timeout<=5:
            raise ValueError('operator_timeout_invalid')
        context=ssl_context or ssl.create_default_context()
        if (not isinstance(context,ssl.SSLContext) or context.verify_mode!=ssl.CERT_REQUIRED
                or not context.check_hostname):
            raise ValueError('cloud_tls_verification_required')
        self.base_url=api_url.rstrip('/');self.timeout=timeout;self._bearer_token=bearer_token
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self,*_):return None
        self._opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),
            urllib.request.HTTPSHandler(context=context),NoRedirect())
        self._initialize_protocol()
        self._expected_context=self._identity(self.status())

    def _read(self,path,payload=None):
        headers={'Authorization':'Bearer '+self._bearer_token}
        body=None
        if payload is not None:
            body=json.dumps(payload,allow_nan=False,separators=(',',':')).encode()
            if len(body)>8192:raise OperatorError('operator_request_too_large')
            headers['Content-Type']='application/json'
        request=urllib.request.Request(self.base_url+path,headers=headers,data=body)
        try:
            with self._opener.open(request,timeout=self.timeout) as response:
                data=response.read(self.MAX_RESPONSE_BYTES+1)
                if len(data)>self.MAX_RESPONSE_BYTES:raise OperatorError('operator_response_too_large')
                if response.headers.get_content_type()!='application/json':
                    raise OperatorError('operator_response_invalid')
                return data
        except urllib.error.HTTPError as error:
            try:value=json.loads(error.read(4096))
            except (ValueError,OSError,HTTPException):value={}
            if not isinstance(value,dict):value={}
            if error.code==401:
                reason='controller_owner_unavailable' if value.get('error')=='unauthorized' else 'cloud_authentication_failed'
            elif error.code==403:reason='cloud_access_denied'
            elif 300<=error.code<400:reason='cloud_redirect_refused'
            else:
                reason=next((v for v in (value.get('error'),value.get('message'))
                    if isinstance(v,str) and re.fullmatch(r'[a-z][a-z0-9_]{0,95}',v)), 'cloud_request_rejected')
            raise OperatorError(reason) from None
        except (OSError,urllib.error.URLError,HTTPException):raise OperatorError('operator_request_uncertain') from None

    def _json(self,path,payload=None):
        def pairs(items):
            result={}
            for key,value in items:
                if key in result:raise ValueError('duplicate_json_key')
                result[key]=value
            return result
        def constant(_):raise ValueError('nonfinite_number')
        try:value=json.loads(self._read(path,payload),object_pairs_hook=pairs,parse_constant=constant)
        except ValueError:raise OperatorError('operator_response_invalid') from None
        if not isinstance(value,dict):raise OperatorError('operator_response_invalid')
        return value
