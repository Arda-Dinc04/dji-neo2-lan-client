import {
  BadRequestException, ConflictException, Injectable, ServiceUnavailableException
} from "@nestjs/common";
import { request as httpRequest, type IncomingHttpHeaders } from "node:http";
import { isAbsolute } from "node:path";

export type ControlResource = "status" | "window" | "frame" | "intent" | "camera";
type Reply = { status: number; headers: IncomingHttpHeaders; body: Buffer };
const READS = new Set(["status", "window", "frame"]);
const WRITES = new Set(["intent", "camera"]);
const MAX_JSON = 256 * 1024;
const MAX_FRAME = 4 * 1024 * 1024;
const MAX_COMMAND = 8192;

/** One private SSH Unix socket, with unchanged PC tickets and command bytes. */
@Injectable()
export class DroneControlGateway {
  private token: string | null = null;
  private tokenRequest: Promise<string> | null = null;
  private outstanding = 0;
  private mutation = false;
  private landing = false;
  private readonly socket = process.env.NEO2_CONTROL_SOCKET || "/run/neo2-control/flight.sock";
  private readonly host = process.env.NEO2_CONTROL_HOST || "127.0.0.1:8794";

  constructor() {
    if (!isAbsolute(this.socket) || this.socket.length > 240 || this.socket.includes("\0")
        || this.socket.split("/").includes("..")
        || !/^(127\.0\.0\.1|localhost):[1-9][0-9]{0,4}$/.test(this.host)
        || Number(this.host.split(":")[1]) > 65535) {
      throw new Error("Invalid private PC controller configuration");
    }
  }

  async forward(method: "GET" | "POST", resource: ControlResource, body?: Buffer): Promise<Reply> {
    if (process.env.NEO2_CONTROL_ENABLED !== "true") {
      throw new ServiceUnavailableException("pc_controller_disabled");
    }
    if (!(method === "GET" ? READS : WRITES).has(resource)) {
      throw new BadRequestException("invalid_controller_resource");
    }
    let isLanding = false;
    if (method === "POST") {
      if (!Buffer.isBuffer(body) || body.length < 1 || body.length > MAX_COMMAND) {
        throw new BadRequestException("invalid_controller_body");
      }
      // Inspect priority only. Forward the original bytes, including duplicate
      // JSON keys, so the PC remains responsible for its strict JSON contract.
      try { isLanding = resource === "intent" && JSON.parse(body.toString("utf8"))?.action === "land"; }
      catch { throw new BadRequestException("invalid_controller_json"); }
      if (isLanding ? this.landing : this.mutation || this.landing) {
        throw new ConflictException("controller_mutation_busy");
      }
      if (isLanding) this.landing = true;
      else this.mutation = true;
    }
    try {
      const token = await this.controllerToken();
      const reply = await this.exchange(method, `/api/remote-flight/${resource}`, body,
        resource === "frame" ? MAX_FRAME : MAX_JSON, token);
      if (reply.status === 401) this.token = null;
      if (reply.status >= 300 && reply.status < 400) {
        throw new ServiceUnavailableException("pc_controller_redirect_refused");
      }
      return reply;
    } finally {
      if (method === "POST") {
        if (isLanding) this.landing = false;
        else this.mutation = false;
      }
    }
  }

  private async controllerToken(): Promise<string> {
    if (this.token) return this.token;
    if (!this.tokenRequest) {
      this.tokenRequest = (async () => {
        const page = await this.exchange("GET", "/", undefined, 64 * 1024);
        const match = page.status === 200 &&
          /<meta name="neo2-token" content="([0-9a-f]{64})"/.exec(page.body.toString("utf8"));
        if (!match) throw new ServiceUnavailableException("pc_controller_unavailable");
        this.token = match[1];
        return this.token;
      })();
    }
    const pending = this.tokenRequest;
    try { return await pending; }
    finally { if (this.tokenRequest === pending) this.tokenRequest = null; }
  }

  private exchange(method: string, path: string, body: Buffer | undefined,
                   maximum: number, token?: string): Promise<Reply> {
    if (this.outstanding >= 8) {
      return Promise.reject(new ServiceUnavailableException("pc_controller_busy"));
    }
    this.outstanding++;
    return new Promise<Reply>((resolve, reject) => {
      let settled = false;
      let timer: ReturnType<typeof setTimeout> | undefined;
      const finish = (error?: unknown, value?: Reply) => {
        if (settled) return;
        settled = true;
        if (timer) clearTimeout(timer);
        this.outstanding--;
        if (error) reject(new ServiceUnavailableException("pc_controller_unavailable"));
        else resolve(value!);
      };
      const headers: Record<string, string | number> = { Host: this.host };
      if (token) headers["X-Neo2-Token"] = token;
      if (body) {
        headers["Content-Type"] = "application/json";
        headers["Content-Length"] = body.length;
      }
      // No proxy environment, redirects, connection pool queue or retry.
      const outgoing = httpRequest({ socketPath: this.socket, method, path, headers,
        agent: false, maxHeaderSize: 16 * 1024 }, (response) => {
        const chunks: Buffer[] = [];
        let size = 0;
        response.on("data", (chunk: Buffer) => {
          size += chunk.length;
          if (size > maximum) {
            outgoing.destroy(); response.destroy(); finish(new Error());
          } else chunks.push(chunk);
        });
        response.on("error", finish);
        response.on("aborted", () => finish(new Error()));
        response.on("end", () => finish(undefined, {
          status: response.statusCode || 502, headers: response.headers, body: Buffer.concat(chunks)
        }));
      });
      outgoing.on("error", finish);
      timer = setTimeout(() => { outgoing.destroy(); finish(new Error()); }, 1500);
      outgoing.end(body);
    });
  }
}
