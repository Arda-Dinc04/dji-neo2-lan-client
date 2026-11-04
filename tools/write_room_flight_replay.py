"""Export the recorded double rotation in the native reconstructed room frame."""
import argparse
from pathlib import Path
from neo2_control.room_mvs import write_new
from neo2_control.room_replay import build_replay


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('poses','mission','session-events','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    replay=build_replay(args.poses,args.mission,args.session_events)
    write_new(args.output,replay)
    print(f"{len(replay['samples'])} samples: {args.output.absolute()}")


if __name__=='__main__':main()
