"""Stream recorded RGB inputs with retained packet identities and arrival clocks.

This decoder reads an audited, canonical HEVC copy. It supplies no camera poses
and never opens a drone connection. Availability follows the last encoded input
needed by the decoder, rather than treating a frame's nominal time as receipt.
"""
from dataclasses import dataclass
from contextlib import contextmanager
import hashlib,json,math,os,stat

from .reconstruction import _read
from .video_lineage import _PacketIdentity


@dataclass(frozen=True)
class RecordedRoomFrame:
    decoded_index: int
    nominal_timestamp: float
    recorded_available_at: float
    rgb: object
    metadata: dict


@contextmanager
def _unchanged(stream,before):
    witness=os.dup(stream.fileno())
    try:
        yield
    finally:
        after=os.fstat(witness);os.close(witness)
        fields=('st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns')
        if any(getattr(before,key)!=getattr(after,key) for key in fields):
            raise ValueError('room_stream_canonical_changed_during_decode')


def audited_frames(canonical,lineage,audit,*,first=0,end=None,stride=2,accept=None,
        decoder_threads=2,decoder_thread_type='FRAME'):
    """Yield selected verified pixels; timestamps remain PC availability labels.

    Setup and decoding are unpaced. A caller starts its replay clock after the
    mapper is ready and schedules ``recorded_available_at`` relative to that
    origin. Keeping the original due time exposes any subsequent backlog.
    An optional ``accept(index, available_at)`` predicate can shed selected
    pictures before RGB conversion. Encoded packet and decode-order checks
    still run; pixel checks apply to every yielded picture.
    """
    import av
    if (type(decoder_threads) is not int or not 1<=decoder_threads<=4
            or decoder_thread_type not in ('SLICE','FRAME')):
        raise ValueError('room_stream_decoder_configuration_invalid')
    manifest=json.loads(_read(lineage,64*1024**2))
    source=json.loads(_read(audit,64*1024**2))
    if (manifest.get('format')!='neo2-control/video-frame-lineage/v1'
            or any(manifest['source'][k]!=source['prepared_source'][k] for k in ('sha256','bytes'))
            or manifest['summary']['pictureless_packets']):
        raise ValueError('room_stream_lineage_or_packet_layout_unsupported')
    entries=manifest['frames'];end=len(entries) if end is None else end
    if (type(first) is not int or type(end) is not int or not 0<=first<end<=len(entries)<=60000
            or type(stride) is not int or not 1<=stride<=4):
        raise ValueError('room_stream_frame_bounds_invalid')
    fps=manifest['preparation']['nominal_fps']
    if type(fps) not in (float,int) or not math.isfinite(fps) or not 1<=fps<=60:
        raise ValueError('room_stream_nominal_rate_invalid')
    pictures={p['prepared_picture_ordinal']:p for p in source['pictures']}
    by_packet={};receipts={}
    for index,entry in enumerate(entries):
        picture=pictures[entry['raw_picture_ordinal']];packet=entry['packet'];position=packet['normalized_position']
        if (entry['decoded_index']!=index or position in by_packet
                or packet['source_vcl_start']!=picture['prepared_start']
                or packet['source_vcl_body_sha256']!=picture['body_sha256']):
            raise ValueError('room_stream_picture_identity_mismatch')
        delivered=picture['last_byte_delivered_monotonic']
        if type(delivered) not in (int,float) or not math.isfinite(delivered) or delivered<0:
            raise ValueError('room_stream_receipt_clock_invalid')
        by_packet[position]=entry;receipts[packet['index']]=delivered
    # Availability is tied to encoded inputs, independently of decoder thread
    # buffering. All dependencies precede their picture in coded packet order.
    packet_available={};last_available=0.
    for index in sorted(receipts):
        last_available=max(last_available,receipts[index]);packet_available[index]=last_available
    fd=os.open(canonical,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    with os.fdopen(fd,'rb') as stream:
        before=os.fstat(stream.fileno());normal=manifest['preparation']['normalization']
        if not stat.S_ISREG(before.st_mode) or not 0<before.st_size==normal['bytes']<=512*1024**2:
            raise ValueError('room_stream_canonical_size_invalid')
        sha=hashlib.sha256()
        while data:=stream.read(65536):sha.update(data)
        if sha.hexdigest()!=normal['sha256']:raise ValueError('room_stream_canonical_changed')
        stream.seek(0);pending={};decoded=0;packet_index=0
        with _unchanged(stream,before),av.open(stream,format='hevc') as container:
            if len(container.streams)!=1 or container.streams[0].codec_context.name!='hevc':
                raise ValueError('room_stream_not_hevc')
            ctx=container.streams[0].codec_context
            ctx.thread_count=decoder_threads;ctx.thread_type=decoder_thread_type
            ctx.copy_opaque=True;ctx.options={'err_detect':'explode','max_pixels':str(1920*1080)}

            def receive(frame):
                nonlocal decoded
                tag=frame.opaque
                if not isinstance(tag,_PacketIdentity) or tag.index not in pending:
                    raise ValueError('room_stream_decoded_identity_missing')
                identity,entry=pending.pop(tag.index)
                if tag is not identity or entry['decoded_index']!=decoded or frame.is_corrupt:
                    raise ValueError('room_stream_decoded_order_or_integrity_invalid')
                pixels=entry['pixels']
                if frame.width!=pixels['width'] or frame.height!=pixels['height']:
                    raise ValueError('room_stream_frame_dimensions_changed')
                index=decoded;decoded+=1
                if index<first or index>=end or (index-first)%stride:return None
                available=packet_available[tag.index]
                if accept is not None and not accept(index,available):return None
                # Decode and check packet/order identity for every picture, but
                # convert only selected inputs. A contiguous ndarray excludes
                # FFmpeg row padding, matching the retained row-wise RGB digest.
                rgb=frame.to_ndarray(format='rgb24')
                pixel_hash=hashlib.sha256(rgb).hexdigest()
                if pixel_hash!=pixels['sha256']:raise ValueError('room_stream_decoded_pixels_changed')
                return RecordedRoomFrame(index,index/fps,available,rgb,{
                    'decoded_index':index,'raw_picture_ordinal':entry['raw_picture_ordinal'],
                    'source_vcl_body_sha256':entry['packet']['source_vcl_body_sha256'],
                    'rgb_sha256':pixel_hash,'canonical_sha256':normal['sha256'],
                    'original_source':source['source'],'recorded_available_at':available,
                    'recorded_picture_delivery_at':receipts[tag.index],
                    'scope':'Audited pixels; PC delivery of this picture and prior encoded inputs, not camera exposure time'})

            for packet in container.demux(video=0):
                if not packet.size:break
                entry=by_packet.get(packet.pos)
                if (entry is None or entry['packet']['index']!=packet_index
                        or packet.size!=entry['packet']['bytes']
                        or hashlib.sha256(bytes(packet)).hexdigest()!=entry['packet']['sha256']):
                    raise ValueError('room_stream_packet_changed')
                packet_index+=1
                tag=_PacketIdentity(entry['packet']['index'],entry['raw_picture_ordinal'])
                pending[tag.index]=(tag,entry);packet.opaque=tag
                for frame in ctx.decode(packet):
                    item=receive(frame)
                    if item is not None:yield item
                    if decoded>=end:return
            for frame in ctx.decode(None):
                item=receive(frame)
                if item is not None:yield item
                if decoded>=end:return
            raise ValueError('room_stream_ended_before_requested_frame')
