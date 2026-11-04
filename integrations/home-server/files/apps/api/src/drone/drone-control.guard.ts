import {
  type ExecutionContext, Injectable, ServiceUnavailableException, UnauthorizedException
} from "@nestjs/common";
import { createHash } from "node:crypto";
import { performance } from "node:perf_hooks";
import { HomeServerAuthGuard, type HomeServerPrincipal } from "../auth/home-server-auth.guard";
import { SupabaseService } from "../supabase.service";

type Verified = {
  principal: HomeServerPrincipal; expiresAt: number; refreshAt: number; jwtExpiresAt: number
};
const MAX_TOKENS = 32;
const CACHE_MS = 5000;
const REFRESH_MS = 1000;
const VALIDATION_MS = 2000;

/** Keep account verification off the PC's 250 ms command path.
 * This cache is restricted to the control API. Other application guards keep
 * their existing behavior. Revocation is rechecked in the background; an
 * unrefreshed result expires after at most five monotonic seconds.
 */
@Injectable()
export class DroneControlGuard extends HomeServerAuthGuard {
  private readonly verified = new Map<string, Verified>();
  private readonly pending = new Map<string, Promise<void>>();

  constructor(supabase: SupabaseService) { super(supabase); }
  protected clock() { return performance.now(); }
  protected wall() { return Date.now(); }

  async canActivate(context: ExecutionContext): Promise<boolean> {
    const request = context.switchToHttp().getRequest();
    const authorization = request.headers.authorization;
    if (typeof authorization !== "string" || authorization.length > 8192
        || !authorization.startsWith("Bearer ")) throw new UnauthorizedException("Missing bearer token");
    const token = authorization.slice(7).trim();
    if (!token) throw new UnauthorizedException("Empty bearer token");
    const key = createHash("sha256").update(token).digest("hex");
    const now = this.clock();
    for (const [id, value] of this.verified) {
      if (now >= value.expiresAt || this.wall() >= value.jwtExpiresAt) this.verified.delete(id);
    }
    let cached = this.verified.get(key);
    if (cached) {
      request.homeServerUser = { ...cached.principal };
      if (now >= cached.refreshAt && !this.pending.has(key)) {
        void this.validate(context, key, token).catch(() => {});
      }
      return true;
    }
    await (this.pending.get(key) || this.validate(context, key, token));
    cached = this.verified.get(key);
    if (!cached || this.clock() >= cached.expiresAt || this.wall() >= cached.jwtExpiresAt) {
      throw new UnauthorizedException("Expired control authentication");
    }
    request.homeServerUser = { ...cached.principal };
    return true;
  }

  private validate(context: ExecutionContext, key: string, token: string): Promise<void> {
    if (this.pending.size >= MAX_TOKENS || (this.verified.size >= MAX_TOKENS && !this.verified.has(key))) {
      return Promise.reject(new ServiceUnavailableException("control_auth_busy"));
    }
    const started = this.clock();
    const operation = super.canActivate(context);
    const job = (async () => {
      let timer: ReturnType<typeof setTimeout> | undefined;
      try {
        const timeout = new Promise<never>((_, reject) => {
          timer = setTimeout(() => reject(new ServiceUnavailableException("control_auth_unavailable")), VALIDATION_MS);
        });
        if (!await Promise.race([operation, timeout])) throw new UnauthorizedException("Invalid control authentication");
        const request = context.switchToHttp().getRequest();
        // Only inspect expiry after the existing guard verified this token and
        // linked its user to the configured administrator. Parsing is not auth.
        const claims = JSON.parse(Buffer.from(token.split(".")[1], "base64url").toString("utf8"));
        if (typeof claims.exp !== "number" || !Number.isFinite(claims.exp)
            || claims.exp * 1000 <= this.wall() || !request.homeServerUser) {
          throw new UnauthorizedException("Invalid control token expiry");
        }
        this.verified.set(key, { principal: { ...request.homeServerUser },
          expiresAt: Math.min(started + CACHE_MS, this.clock() + claims.exp * 1000 - this.wall()),
          refreshAt: started + REFRESH_MS, jwtExpiresAt: claims.exp * 1000 });
      } catch (error) {
        this.verified.delete(key);
        if (error instanceof UnauthorizedException || error instanceof ServiceUnavailableException) throw error;
        throw new UnauthorizedException("Control authentication rejected");
      } finally { if (timer) clearTimeout(timer); }
    })();
    this.pending.set(key, job);
    // A timed-out Supabase operation may still be in flight. Keep its slot
    // until it really settles, rather than allowing unbounded abandoned work.
    void operation.then(() => this.pending.delete(key), () => this.pending.delete(key));
    return job;
  }
}
