"""Build a textured GLB from retained, source-verified COLMAP geometry on CPU."""
import argparse,json
from pathlib import Path
from neo2_control.room_mvs import prepare_workspace,run_stage,inspect_glb,pack_glb,write_new

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('prepared','model','toolchain','output'):
        parser.add_argument('--'+name,required=True,type=Path)
    args=parser.parse_args()
    prepare_workspace(args.prepared,args.model,args.output)
    for stage in ('import','dense','mesh','texture'):
        print(json.dumps(run_stage(args.output,args.toolchain,stage)),flush=True)
    pack_glb(args.output/'scene_textured.glb',args.output/'room.glb')
    result=inspect_glb(args.output/'room.glb')
    write_new(args.output/'textured-asset.json',result)
    print(json.dumps(result),flush=True)

if __name__=='__main__':main()
