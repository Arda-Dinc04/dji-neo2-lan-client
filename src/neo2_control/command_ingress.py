"""PC-issued, single-use operator windows; reading one grants no flight lease.

Only this PC's monotonic clock is meaningful in a window. The signed carrier
is verified by HTTP; the immutable context travels unchanged through private
IPC and the final command queues. Transport RPC deadlines stay separate.
"""
from dataclasses import asdict, dataclass
import hashlib
import hmac
import json
import math
import re
import secrets
import threading
import time


WINDOW_SECONDS = .250
MAX_SEQUENCE = (1 << 53) - 1
_UNSPECIFIED = object()


@dataclass(frozen=True)
class CommandIngress:
    owner_epoch: str
    connection_generation: str | None
    mutation_sequence: int
    issued_at: float
    expires_at: float

    def __post_init__(self):
        if (not isinstance(self.owner_epoch,str)
                or re.fullmatch('[0-9a-f]{48}',self.owner_epoch) is None
                or (self.connection_generation is not None and (
                    not isinstance(self.connection_generation,str)
                    or re.fullmatch('[0-9a-f]{24}',self.connection_generation) is None))
                or type(self.mutation_sequence) is not int
                or not 1 <= self.mutation_sequence <= MAX_SEQUENCE
                or any(type(v) not in (float,int) or not math.isfinite(v)
                       for v in (self.issued_at,self.expires_at))
                or not 0 <= self.issued_at < self.expires_at
                or self.expires_at-self.issued_at > WINDOW_SECONDS+1e-9):
            raise ValueError('operator_request_invalid')

    def validate(self,now,*,owner_epoch=_UNSPECIFIED,connection_generation=_UNSPECIFIED):
        if type(now) not in (float,int) or not math.isfinite(now):
            raise ValueError('operator_request_invalid')
        if not self.issued_at <= now < self.expires_at:
            raise ValueError('operator_request_expired')
        if ((owner_epoch is not _UNSPECIFIED and self.owner_epoch != owner_epoch)
                or (connection_generation is not _UNSPECIFIED
                    and self.connection_generation != connection_generation)):
            raise ValueError('operator_request_generation_changed')
        return self

    def as_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls,value):
        if not isinstance(value,dict) or set(value) != set(cls.__dataclass_fields__):
            raise ValueError('operator_request_invalid')
        return cls(**value)


class CommandWindows:
    """Bounded state: secret, issuance counter and last consumed sequence only."""

    context_type = CommandIngress

    def __init__(self,*,clock=time.monotonic):
        self.clock=clock
        self._secret=secrets.token_bytes(32)
        self._issued=self._consumed=0
        self._lock=threading.Lock()

    def issue(self,owner_epoch,connection_generation):
        return self._issue(owner_epoch,connection_generation,WINDOW_SECONDS)

    def _issue(self,owner_epoch,connection_generation,seconds):
        with self._lock:
            if self._issued >= MAX_SEQUENCE:raise ValueError('operator_window_sequence_exhausted')
            now=self.clock()
            context=self.context_type(owner_epoch,connection_generation,self._issued+1,
                now,now+seconds)
            self._issued=context.mutation_sequence
            payload=json.dumps(context.as_dict(),sort_keys=True,separators=(',',':'),
                allow_nan=False).encode().hex()
            signature=hmac.new(self._secret,payload.encode('ascii'),hashlib.sha256).hexdigest()
            return {'ticket':payload+'.'+signature,'mutation_sequence':context.mutation_sequence,
                'expires_in_seconds':seconds,'owner_epoch':owner_epoch,
                'connection_generation':connection_generation}

    def consume(self,value,*,owner_epoch,connection_generation):
        if (not isinstance(value,dict) or set(value) != {'ticket','mutation_sequence',
                'expected_owner_epoch','expected_connection_generation'}
                or not isinstance(value['ticket'],str) or len(value['ticket'])>2048
                or type(value['mutation_sequence']) is not int):
            raise ValueError('operator_window_required')
        ticket=value['ticket']
        if re.fullmatch('[0-9a-f]+\\.[0-9a-f]{64}',ticket) is None:
            raise ValueError('operator_window_invalid')
        payload,signature=ticket.split('.')
        expected=hmac.new(self._secret,payload.encode('ascii'),hashlib.sha256).hexdigest()
        if not hmac.compare_digest(signature,expected):raise ValueError('operator_window_invalid')
        try:context=self.context_type.from_dict(json.loads(bytes.fromhex(payload)))
        except (ValueError,UnicodeError,TypeError):raise ValueError('operator_window_invalid') from None
        if value['mutation_sequence'] != context.mutation_sequence:
            raise ValueError('operator_window_invalid')
        with self._lock:
            # The gesture's original context must match the signed window.
            # Obtaining a window from a replacement owner cannot adopt it.
            context.validate(self.clock(),owner_epoch=value['expected_owner_epoch'],
                connection_generation=value['expected_connection_generation'])
            context.validate(self.clock(),owner_epoch=owner_epoch,
                connection_generation=connection_generation)
            if context.mutation_sequence <= self._consumed:
                raise ValueError('operator_request_replayed')
            # A business refusal cannot make the same operation replayable.
            self._consumed=context.mutation_sequence
        return context
