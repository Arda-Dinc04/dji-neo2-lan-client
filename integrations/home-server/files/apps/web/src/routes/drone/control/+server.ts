import { base } from "$app/paths";
import { error } from "@sveltejs/kit";
import { homeServerApi } from "$lib/api";
import template from "$lib/server/neo2-flight.html?raw";
import type { RequestHandler } from "./$types";

const attribute = (value: string) => value.replace(/[&<>"']/g,
  character => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[character]!);

export const GET: RequestHandler = async ({ locals, fetch }) => {
  const token = locals.session?.access_token;
  if (!token) throw error(401, "Authentication required");
  // The cookie hook proves a session. The existing API additionally proves
  // that it belongs to this console's configured administrator.
  try {
    const profile = await homeServerApi<{ user?: { role?: string } }>("/me", { token, fetch });
    if (profile.user?.role !== "admin") throw new Error();
  } catch { throw error(403, "Access denied"); }
  let html = template.replace("__NEO2_TOKEN__", attribute(token))
    .replace('name="neo2-api-prefix" content="/api/flight"',
      `name="neo2-api-prefix" content="${base}/api/drone/control"`)
    .replace('name="neo2-auth" content="local"', 'name="neo2-auth" content="bearer"')
    .replace('name="neo2-command-mode" content="native"', 'name="neo2-command-mode" content="remote-target"')
    .replaceAll("Contrôle PC", "Contrôle du drone");
  for (const file of ["favicon.svg", "camera.css", "flight.css", "flight.js"]) {
    html = html.replaceAll(`"/${file}"`, `"${base}/neo2/${file}"`);
  }
  return new Response(html, { headers: {
    "Content-Type": "text/html; charset=utf-8", "Cache-Control": "no-store",
    "Vary": "Cookie", "Referrer-Policy": "no-referrer", "X-Content-Type-Options": "nosniff",
    "Content-Security-Policy": "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'self'; base-uri 'none'; form-action 'none'"
  } });
};
