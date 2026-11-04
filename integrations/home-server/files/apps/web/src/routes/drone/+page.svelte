<script lang="ts">
  import { base } from "$app/paths";
  let { data } = $props();
</script>

<svelte:head><title>Drone · Example Home Server</title></svelte:head>

{#if data.allowed}
  <iframe src={`${base}/drone/control`} title="Caméra et commandes du DJI Neo 2"></iframe>
  <p class="muted"><a href={`${base}/drone/video`}>Ouvrir le lecteur HLS</a></p>
{:else}
  <p role="alert">{data.accessError || "Accès au drone indisponible."}</p>
{/if}

<style>
  iframe { width: 100%; min-height: 980px; height: calc(100vh - 150px); border: 0; border-radius: 18px; }
  .muted { text-align: right; color: var(--muted); }
  @media (max-width: 900px) { iframe { min-height: 1400px; } }
</style>
