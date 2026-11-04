"""Measured FC height input for generic visual scale estimation; no flight API."""
from bisect import bisect_right
import json
import math
from pathlib import Path


def _finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def status_altitude_observation(status):
    """Use the existing controller status received before reading a preview."""
    telemetry=status.get('telemetry') or {}
    height=telemetry.get('altitude_m');received=telemetry.get('motion_received_at');available=status.get('now')
    if (status.get('synthetic') is not False or telemetry.get('synthetic') is not False
            or not all(map(_finite,(height,received,available)))
            or not 0<=received<=available or available-received>.5):return None
    result={'kind':'measured_relative_altitude','value_m':height,
        'received_at':received,'available_at':available,'quantization_m':.1,
        'source':'FC OSD relative altitude','physical_accuracy_verified':False}
    yaw=telemetry.get('yaw_deg')
    if _finite(yaw) and -180<=yaw<=180:result['body_yaw_deg']=yaw
    return result


class RecordedAltitude:
    """Causal receipt-clock join, without mission phases, targets or fitted poses."""
    def __init__(self,session_events):
        rows={};origin=None
        with Path(session_events).open() as stream:
            for line in stream:
                event=json.loads(line)
                if event.get('event')!='telemetry_updated':continue
                elapsed=event.get('elapsed_seconds')
                if not _finite(elapsed):raise ValueError('room_altitude_elapsed_invalid')
                for observation in event['telemetry'].get('observations',[]):
                    received=observation.get('received_at');age=observation.get('age_seconds')
                    if not all(map(_finite,(received,age))) or received<0 or age<0:continue
                    candidate=received+age-elapsed
                    if origin is None:origin=candidate
                    elif abs(origin-candidate)>1e-6:raise ValueError('room_altitude_pc_clock_mismatch')
                    altitude=observation.get('fields',{}).get('altitude',{})
                    source=observation.get('source',{})
                    if (altitude.get('valid') is not True or altitude.get('unit')!='m'
                            or altitude.get('reference')!='relative' or not _finite(altitude.get('value'))
                            # Keep the Wi-Fi FC->14 stream separate from other
                            # forwarded recipients batched at the same PC time.
                            or (source.get('sender'),source.get('receiver'),source.get('command_set'),
                                source.get('command_id'))!=(3,14,3,67)):
                        continue
                    row={'kind':'measured_relative_altitude','value_m':altitude['value'],
                        'received_at':received,'available_at':received+age,'quantization_m':.1,
                        'source':'FC OSD relative altitude','physical_accuracy_verified':False}
                    attitude=observation.get('fields',{}).get('attitude',{})
                    angles=attitude.get('value')
                    if (attitude.get('valid') is True and attitude.get('unit')=='degree'
                            and isinstance(angles,(tuple,list)) and len(angles)==3
                            and all(_finite(v) and -180<=v<=180 for v in angles)):
                        row['body_yaw_deg']=angles[2]
                    previous=rows.get(received)
                    if previous is not None and previous['value_m']!=row['value_m']:
                        raise ValueError('room_altitude_conflicting_observation')
                    if previous is None or row['available_at']<previous['available_at']:rows[received]=row
        self.rows=sorted(rows.values(),key=lambda row:(row['available_at'],row['received_at']))
        self.times=[row['available_at'] for row in self.rows]

    def at(self,image_received_at,*,replayed_at=None):
        """Past data only; optionally preserve deltas on a paced replay clock."""
        if not _finite(image_received_at) or (replayed_at is not None and not _finite(replayed_at)):
            raise ValueError('room_altitude_image_clock_invalid')
        i=bisect_right(self.times,image_received_at)-1
        if i<0:return None
        row=self.rows[i]
        if not 0<=image_received_at-row['received_at']<=.5:return None
        result=dict(row)
        if replayed_at is not None:
            delta=replayed_at-image_received_at
            result.update(original_received_at=row['received_at'],original_available_at=row['available_at'])
            result['received_at']+=delta;result['available_at']+=delta
        return result
