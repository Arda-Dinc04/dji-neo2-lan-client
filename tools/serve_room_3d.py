"""Display an acquisition's changing textured geometry on loopback only."""
import argparse
from pathlib import Path
from neo2_control.room_live import create_server


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--acquisition',type=Path,required=True)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8795)
    args=parser.parse_args()
    if not 0<=args.port<=65535:raise ValueError('room_viewer_port_invalid')
    with create_server(args.acquisition,args.bundle,port=args.port) as server:
        print(f'http://127.0.0.1:{server.server_port}',flush=True)
        try:server.serve_forever()
        except KeyboardInterrupt:pass


if __name__=='__main__':main()
