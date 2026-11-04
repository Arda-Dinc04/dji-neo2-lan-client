"""Adapt the recorded Neo2 flight to Spatial Capture's independent scene viewer."""
from pathlib import Path
import argparse,json
from neo2_control.room_mvs import digest
from spatial_capture.scene import write_scene


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('mesh','replay','bundle','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();replay=json.loads(args.replay.read_text())
    if replay['format']!='neo2-control/room-flight-replay/v1' or not replay['samples']:
        raise ValueError('room_scene_replay_invalid')
    first,second=replay['rotation_intervals']
    boundaries=[0,first['start_seconds'],first['end_seconds'],second['start_seconds'],
        second['end_seconds'],replay['duration_seconds']]
    labels=['Décollage et montée','Premier 360° · palier 1 m','Montée vers le second palier',
        'Second 360° · palier 1,50 m','Atterrissage']
    camera_replay={'format':'spatial-capture/camera-replay/v1',
        'duration_seconds':replay['duration_seconds'],'up_axis':replay['up_axis'],
        'model_units_per_source_metre':replay['model_units_per_FC_metre'],
        'height_label':'Altitude relative FC','source':replay['source'],
        'metadata':{'rotation_intervals':replay['rotation_intervals'],'scale_source':replay['scale_source']},
        'vehicle_visual':{'name':'DJI Neo 2','kind':'guarded_quadcopter',
            'dimensions_metres':[.147,.171,.041],'propeller_diameter_metres':.05588,
            'approximation':True,'geometry_source':'Original procedural illustration, not manufacturer CAD',
            'dimensions_source':'https://www.dji.com/se/neo-2/specs?set_region=SE',
            'shape_source':'https://store.dji.com/sg/product/dji-neo-2',
            'propeller_source':'https://repair.dji.com/help/content?customId=01700006559&lang=en&paperDocType=ARTICLE&re=US&spaceId=17'},
        'samples':[{k:s[k] for k in ('seconds','position','world_from_camera_R')}|
            {'source_height_m':s['FC_relative_altitude_m']} for s in replay['samples']],
        'phases':[{'start_seconds':start,'end_seconds':end,'label':label}
            for start,end,label in zip(boundaries,boundaries[1:],labels)]}
    note=('Modèle expérimental : les surfaces absentes restent inconnues ; l’orange signale une surface sans '
        'texture source. Échelle indicative ajustée sur l’altitude relative FC, précision physique non vérifiée. '
        'Le rejeu situe la caméra dans le repère reconstruit ; décollage et descente sont estimés depuis la '
        'télémétrie. La synchronisation d’exposition et le décalage caméra–drone restent à établir.')
    audit=write_scene(args.mesh,camera_replay,args.bundle,args.output,
        title='Chambre reconstruite · rejeu du double 360°',note=note,
        source_identity={'replay_sha256':digest(args.replay),'source':replay['source']})
    print(json.dumps({'output':str(args.output.absolute()),**audit['asset']}))


if __name__=='__main__':main()
