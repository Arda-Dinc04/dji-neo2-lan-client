import AppKit
import CoreLocation
import CoreWLAN
import Foundation

// Permission is used only to read current Wi-Fi identity. No location fix,
// network scan, association, keychain read or device request is performed.
final class WifiCheck: NSObject, CLLocationManagerDelegate {
    let manager = CLLocationManager()
    let interface: String
    var attempts = 0
    var finished = false
    init(_ interface: String) { self.interface = interface; super.init(); manager.delegate = self }
    func finish(_ value: [String: Any], _ code: Int32) {
        if finished { return }; finished = true
        if let data = try? JSONSerialization.data(withJSONObject: value, options: [.sortedKeys]) {
            FileHandle.standardOutput.write(data); FileHandle.standardOutput.write(Data([10]))
        }
        exit(code)
    }
    func begin() {
        switch manager.authorizationStatus {
        case .notDetermined: manager.requestWhenInUseAuthorization()
        case .authorizedAlways, .authorizedWhenInUse: read()
        default: finish(["error": "macos_wifi_location_permission_required"], 2)
        }
    }
    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) { begin() }
    func read() {
        guard let wifi = CWWiFiClient.shared().interface(withName: interface), wifi.powerOn() else {
            finish(["error": "macos_wifi_interface_unavailable"], 2); return
        }
        if let ssid = wifi.ssidData(), let bssid = wifi.bssid() {
            finish(["ssid_hex": ssid.map { String(format: "%02x", $0) }.joined(), "bssid": bssid], 0)
        } else if attempts < 5 {
            attempts += 1
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) { self.read() }
        } else { finish(["error": "macos_wifi_identity_unavailable"], 2) }
    }
}

guard CommandLine.arguments.count == 2 else { exit(2) }
let app = NSApplication.shared
app.setActivationPolicy(.accessory)
app.finishLaunching()
let checker = WifiCheck(CommandLine.arguments[1])
DispatchQueue.main.async { checker.begin() }
DispatchQueue.main.asyncAfter(deadline: .now() + 25) {
    checker.finish(["error": "macos_wifi_permission_wait_timeout"], 2)
}
app.run()
