import type { PageServerLoad } from "./$types";
export const load: PageServerLoad = async ({ parent }) => {
  const { homeServerUser, accessError } = await parent();
  return { allowed: homeServerUser?.role === "admin", accessError };
};
