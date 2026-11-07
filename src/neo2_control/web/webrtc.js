/* Local WHEP playback only. No aircraft commands, external ICE servers or SDP logs. */
const MAX_SDP_BYTES = 65536;
const STALE_MS = 3000;
const RETRY_MS = 2500;
const MAX_ATTEMPTS = 3;
const now = () => performance.now();

const numeric = value => typeof value === 'number' && Number.isFinite(value) && value >= 0 && value <= Number.MAX_SAFE_INTEGER ? value : null;
const difference = (value, previous) => value != null && previous != null && value >= previous ? value - previous : null;

// Deliberately retain numeric receiver measurements only. Raw RTCStats contain
// addresses, track IDs and other identifiers that do not belong in an export.
export class ReceiverDiagnostics {
  constructor() { this.reset(0); }
  reset(at) {
    this.origin = at; this.attempt = 0; this.active = false;
    this.previous = this.latest = null; this.history = [];
  }
  begin(at) { this.attempt += 1; this.active = true; this.previous = this.latest = null; }
  end() { this.active = false; }
  sample(report, quality, at) {
    const rows = [...report.values()].filter(row => row.type === 'inbound-rtp' && (row.kind || row.mediaType) === 'video');
    if (rows.length !== 1) return;
    const row = rows[0];
    const current = {at, id: row.id, decoded: numeric(row.framesDecoded),
      buffer: numeric(row.jitterBufferDelay), emitted: numeric(row.jitterBufferEmittedCount),
      decode: numeric(row.totalDecodeTime)};
    const before = this.previous?.id === current.id ? this.previous : null;
    const seconds = before ? (at - before.at) / 1000 : 0;
    const interval = seconds > 0 && seconds <= 10;
    const frames = before ? difference(current.decoded, before.decoded) : null;
    const emitted = before ? difference(current.emitted, before.emitted) : null;
    const buffer = before ? difference(current.buffer, before.buffer) : null;
    const decode = before ? difference(current.decode, before.decode) : null;
    const mean = (total, count) => interval && total != null && count > 0 ? numeric(total * 1000 / count) : null;
    const sample = {elapsed_seconds: Math.max(0, (at - this.origin) / 1000), attempt: this.attempt,
      decoded_fps: interval && frames != null ? numeric(frames / seconds) : null,
      browser_buffer_ms: mean(buffer, emitted), browser_decode_ms: mean(decode, frames),
      frames_decoded: current.decoded, browser_total_frames: numeric(quality?.totalVideoFrames),
      browser_dropped_frames: numeric(quality?.droppedVideoFrames),
      width: numeric(row.frameWidth), height: numeric(row.frameHeight)};
    this.previous = current; this.latest = {at, sample}; this.history.push(sample);
    if (this.history.length > 120) this.history.shift();
  }
  snapshot(at) {
    const age = this.latest ? Math.max(0, (at - this.latest.at) / 1000) : null;
    const fresh = this.active && age != null && age <= 3;
    const value = this.latest?.sample || {};
    return {...value, available: fresh, sample_age_seconds: age, sample_count: this.history.length,
      decoded_fps: fresh ? value.decoded_fps ?? null : null,
      browser_buffer_ms: fresh ? value.browser_buffer_ms ?? null : null,
      browser_decode_ms: fresh ? value.browser_decode_ms ?? null : null};
  }
  export(at, sourceMode) {
    return {schema: 'neo2.playback-diagnostics.v1',
      source_mode: ['live_aircraft', 'synthetic'].includes(sourceMode) ? sourceMode : 'unavailable',
      scope: 'local_webrtc_receiver', camera_to_screen_delay_ms: null,
      maximum_samples: 120, current: this.snapshot(at), samples: this.history.map(sample => ({...sample}))};
  }
}

async function boundedSdp(response) {
  if (Number(response.headers.get('Content-Length')) > MAX_SDP_BYTES) throw new Error('answer_too_large');
  if (!response.body?.getReader) {
    const text = await response.text();
    if (new TextEncoder().encode(text).length > MAX_SDP_BYTES) throw new Error('answer_too_large');
    return text;
  }
  const reader = response.body.getReader();
  const chunks = [];
  let size = 0;
  try {
    while (true) {
      const {value, done} = await reader.read();
      if (done) break;
      size += value.byteLength;
      if (size > MAX_SDP_BYTES) throw new Error('answer_too_large');
      chunks.push(value);
    }
  } catch (error) {
    await reader.cancel().catch(() => {});
    throw error;
  } finally { reader.releaseLock(); }
  const bytes = new Uint8Array(size);
  let offset = 0;
  for (const chunk of chunks) { bytes.set(chunk, offset); offset += chunk.byteLength; }
  return new TextDecoder().decode(bytes);
}

export class WhepPreview {
  constructor({video, onChange = () => {}, fetchImpl = globalThis.fetch.bind(globalThis), PeerConnection = globalThis.RTCPeerConnection, clock = now}) {
    this.video = video;
    this.onChange = onChange;
    this.fetch = fetchImpl;
    this.PeerConnection = PeerConnection;
    this.clock = clock;
    this.context = null;
    this.mode = 'auto';
    this.binding = null;
    this.session = null;
    this.serial = 0;
    this.attempts = 0;
    this.retryAt = 0;
    this.closed = false;
    this.receiver = new ReceiverDiagnostics();
    this.value = {state: 'waiting', fresh: false, reason: null, attempts: 0};
    video.muted = true;
    video.autoplay = true;
    video.playsInline = true;
    this.timer = setInterval(() => this._tick(), 250);
  }

  get snapshot() { return {...this.value}; }
  get diagnostics() { return this.receiver.snapshot(this.clock()); }
  exportDiagnostics(sourceMode) { return this.receiver.export(this.clock(), sourceMode); }

  _publish(state, reason = null) {
    const value = {state, fresh: state === 'live', reason, attempts: this.attempts};
    if (JSON.stringify(value) === JSON.stringify(this.value)) return;
    this.value = value;
    this.onChange(this.snapshot);
  }

  update(context) {
    if (this.closed) return;
    // Equality only: binding values (especially the token) never enter logs or DOM.
    const binding = [context.token, context.sessionGeneration, context.generation];
    if (!this.binding || binding.some((value, i) => value !== this.binding[i])) {
      this._stopActive();
      this.receiver.reset(this.clock());
      this.binding = binding;
      this.attempts = 0;
      this.retryAt = 0;
    }
    this.context = {...context};
    this._tick();
  }

  setMode(mode) {
    if (!['auto', 'jpeg'].includes(mode) || mode === this.mode || this.closed) return;
    this.mode = mode;
    this._stopActive();
    this._tick();
  }

  _eligible() {
    const c = this.context;
    return c?.connected === true && c.live === true && c.enabled === true && c.ready === true &&
      c.generation != null && c.sessionGeneration != null && typeof c.token === 'string' && c.token.length > 0;
  }

  _tick() {
    if (this.closed) return;
    if (this.mode === 'jpeg') { this._stopActive(); this._publish('disabled', 'jpeg_selected'); return; }
    if (!this.context?.enabled) { this._stopActive(); this._publish('disabled', 'bridge_disabled'); return; }
    if (!this._eligible()) { this._stopActive(); this._publish('waiting', this.context?.error || null); return; }
    if (!this.PeerConnection) { this._publish('fallback', 'unsupported'); return; }
    const s = this.session;
    if (s) {
      const time = this.clock();
      // currentTime advancement is a fallback for browsers without frame callbacks.
      const position = this.video.currentTime;
      if (!this.video.requestVideoFrameCallback && this.video.readyState >= 2 && !this.video.paused && !this.video.ended && Number.isFinite(position)) {
        if (s.lastTime != null && position > s.lastTime) this._frame(s);
        s.lastTime = position;
      }
      if (s.frameAt != null && time - s.frameAt > STALE_MS) this._fail(s, 'stalled');
      else if (s.frameAt == null && time - s.started > 12000) this._fail(s, 'no_frame');
      if (this._active(s)) void this._sampleStats(s);
      return;
    }
    if (this.attempts >= MAX_ATTEMPTS) return;
    if (this.clock() >= this.retryAt) void this._connect();
  }

  _active(s) { return !this.closed && this.session === s && s.serial === this.serial; }

  async _sampleStats(s) {
    if (typeof s.pc?.getStats !== 'function' || s.statsPending || this.clock() < s.nextStatsAt) return;
    s.statsPending = true; s.nextStatsAt = this.clock() + 1000;
    try {
      const report = await s.pc.getStats();
      if (!this._active(s)) return;
      let quality;
      try { quality = this.video.getVideoPlaybackQuality?.(); } catch { /* Optional browser API. */ }
      this.receiver.sample(report, quality, this.clock());
    } catch { /* Missing receiver stats must never stop camera playback. */ }
    finally { s.statsPending = false; }
  }

  _frame(s) {
    if (!this._active(s)) return;
    s.frameAt = this.clock();
    this._publish('live');
  }

  _watchFrames(s) {
    if (!this._active(s) || !this.video.requestVideoFrameCallback) return;
    s.frameRequest = this.video.requestVideoFrameCallback((_time, metadata) => {
      if (!this._active(s)) return;
      if (this.video.readyState >= 2 && !this.video.paused && !this.video.ended &&
          (s.presented == null || metadata.presentedFrames > s.presented || metadata.mediaTime > s.mediaTime)) this._frame(s);
      s.presented = metadata.presentedFrames;
      s.mediaTime = metadata.mediaTime;
      this._watchFrames(s);
    });
  }

  _gather(s) {
    if (s.pc.iceGatheringState === 'complete') return Promise.resolve();
    return new Promise((resolve, reject) => {
      let timer;
      const finish = error => {
        clearTimeout(timer);
        s.pc.removeEventListener('icegatheringstatechange', changed);
        s.controller.signal.removeEventListener('abort', aborted);
        error ? reject(error) : resolve();
      };
      const changed = () => { if (s.pc.iceGatheringState === 'complete') finish(); };
      const aborted = () => finish(new Error('cancelled'));
      s.pc.addEventListener('icegatheringstatechange', changed);
      s.controller.signal.addEventListener('abort', aborted, {once: true});
      timer = setTimeout(() => finish(new Error('ice_gather_timeout')), 3000);
      changed();
    });
  }

  async _connect() {
    if (this.session || !this._eligible() || this.mode !== 'auto' || this.closed) return;
    this.attempts += 1;
    const s = {serial: ++this.serial, token: this.context.token, controller: new AbortController(), location: null,
      pc: null, frameAt: null, started: this.clock(), lastTime: null, frameRequest: null, nextStatsAt: 0, statsPending: false};
    this.session = s;
    this.receiver.begin(this.clock());
    this._publish('connecting');
    try {
      const pc = s.pc = new this.PeerConnection({iceServers: [], bundlePolicy: 'max-bundle'});
      pc.addTransceiver('video', {direction: 'recvonly'});
      pc.onconnectionstatechange = () => {
        if (this._active(s) && ['failed', 'disconnected', 'closed'].includes(pc.connectionState)) this._fail(s, 'connection_failed');
      };
      pc.oniceconnectionstatechange = () => {
        if (this._active(s) && ['failed', 'disconnected'].includes(pc.iceConnectionState)) this._fail(s, 'ice_failed');
      };
      pc.ontrack = event => {
        if (!this._active(s) || event.track.kind !== 'video') return;
        this.video.srcObject = event.streams[0] || new MediaStream([event.track]);
        event.track.onended = () => { if (this._active(s)) this._fail(s, 'track_ended'); };
        this._watchFrames(s);
        Promise.resolve(this.video.play()).catch(() => { if (this._active(s)) this._fail(s, 'playback_blocked'); });
      };
      const offer = await pc.createOffer();
      if (!this._active(s)) return;
      await pc.setLocalDescription(offer);
      if (!this._active(s)) return;
      await this._gather(s);
      if (!this._active(s)) return;
      const sdp = pc.localDescription?.sdp;
      if (typeof sdp !== 'string' || !sdp.length || new TextEncoder().encode(sdp).length > MAX_SDP_BYTES) throw new Error('offer_invalid');
      const response = await this.fetch('/api/camera/webrtc', {method: 'POST', headers: {
        'Content-Type': 'application/sdp', 'X-Neo2-Token': s.token}, body: sdp,
        credentials: 'same-origin', redirect: 'error', cache: 'no-store', signal: s.controller.signal});
      const location = response.headers.get('Location');
      if (response.status !== 201 || !/^\/api\/camera\/webrtc\/[a-f0-9]{1,128}$/i.test(location || '')) throw new Error('offer_rejected');
      s.location = location;
      if (!this._active(s)) { this._delete(s); return; }
      const answer = await boundedSdp(response);
      if (!this._active(s)) return;
      if (!answer.trim()) throw new Error('answer_invalid');
      await pc.setRemoteDescription({type: 'answer', sdp: answer});
    } catch (error) {
      if (this._active(s)) this._fail(s, ['ice_gather_timeout', 'answer_too_large', 'offer_invalid', 'offer_rejected', 'answer_invalid'].includes(error.message) ? error.message : 'negotiation_failed');
    }
  }

  _delete(s) {
    if (!s.location || s.deleted) return;
    s.deleted = true;
    // Keep cleanup local and authenticated, including during page teardown.
    void this.fetch(s.location, {method: 'DELETE', headers: {'X-Neo2-Token': s.token},
      credentials: 'same-origin', redirect: 'error', cache: 'no-store', keepalive: true,
      signal: AbortSignal.timeout(3000)}).catch(() => {});
  }

  _stopActive() {
    const s = this.session;
    if (!s) return;
    this.session = null;
    this.receiver.end();
    this.serial += 1;
    s.controller.abort();
    if (s.frameRequest != null && this.video.cancelVideoFrameCallback) this.video.cancelVideoFrameCallback(s.frameRequest);
    if (s.pc) { s.pc.ontrack = null; s.pc.onconnectionstatechange = null; s.pc.oniceconnectionstatechange = null; s.pc.close(); }
    this.video.pause();
    this.video.srcObject = null;
    this._delete(s);
  }

  _fail(s, reason) {
    if (!this._active(s)) return;
    this._stopActive();
    this.retryAt = this.clock() + RETRY_MS;
    this._publish('fallback', reason);
  }

  stop() {
    if (this.closed) return;
    this.closed = true;
    clearInterval(this.timer);
    this._stopActive();
    this._publish('disabled', 'closed');
  }
}
