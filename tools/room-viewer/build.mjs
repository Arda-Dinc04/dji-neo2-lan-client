// Explicit destination keeps generated bundles outside the source tree.
import {build} from 'esbuild';
import {fileURLToPath} from 'node:url';
import {resolve} from 'node:path';
import {readFileSync,existsSync} from 'node:fs';
import {execFileSync} from 'node:child_process';

const output=process.argv[2];
if(!output)throw Error('usage: node tools/room-viewer/build.mjs OUTPUT.js');
const license=readFileSync(new URL('node_modules/three/LICENSE',import.meta.url),'utf8');
const localPython=fileURLToPath(new URL('../../.venv/bin/python',import.meta.url));
const python=process.env.SPATIAL_CAPTURE_PYTHON||(existsSync(localPython)?localPython:'python3');
const source=execFileSync(python,['-c',
  "from importlib.resources import files; print(files('spatial_capture').joinpath('web/room_scene.js'))"],{encoding:'utf8'}).trim();
await build({
  entryPoints:[source],
  nodePaths:[fileURLToPath(new URL('node_modules',import.meta.url))],
  bundle:true,format:'iife',globalName:'SpatialCapture',target:'es2020',minify:true,
  banner:{js:'/*! Three.js 0.180.0\n'+license+'\n*/'},
  legalComments:'inline',outfile:resolve(output)
});
