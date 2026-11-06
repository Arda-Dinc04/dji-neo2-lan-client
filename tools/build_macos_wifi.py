#!/usr/bin/env python3
"""Build a local Wi-Fi identity permission helper using the existing Apple SDK."""
from pathlib import Path
import plistlib
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    if sys.platform != 'darwin':raise SystemExit('This helper uses macOS frameworks.')
    bundle = ROOT/'build/Neo2WifiCheck.app/Contents'
    (bundle/'MacOS').mkdir(parents=True, exist_ok=True)
    purpose = 'Verify the current Wi-Fi network before a DJI Neo 2 camera preview. No location fixes are requested.'
    info = {'CFBundleIdentifier':'org.neo2research.wifi-check', 'CFBundleName':'Neo 2 Wi-Fi Check',
            'CFBundleExecutable':'Neo2WifiCheck', 'CFBundlePackageType':'APPL',
            'CFBundleVersion':'1', 'CFBundleShortVersionString':'1.0', 'LSUIElement':True,
            'NSLocationUsageDescription':purpose, 'NSLocationWhenInUseUsageDescription':purpose}
    (bundle/'Info.plist').write_bytes(plistlib.dumps(info))
    subprocess.run(['/usr/bin/xcrun','swiftc',str(ROOT/'native/MacWifiIdentity.swift'),
                    '-o',str(bundle/'MacOS/Neo2WifiCheck'),'-framework','AppKit',
                    '-framework','CoreLocation','-framework','CoreWLAN'],check=True)
    subprocess.run(['/usr/bin/codesign','--force','--sign','-',str(bundle.parent)],check=True,
                   stdout=subprocess.DEVNULL)
    print('Built local helper: build/Neo2WifiCheck.app')


if __name__ == '__main__':main()
