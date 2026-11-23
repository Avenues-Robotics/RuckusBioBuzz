<script lang="ts">
  /**
   * The auto's mechanism states over time (utils/mechanismStates.ts), one lane each,
   * under the playback bar. Click a span or a change tick to jump the playhead there.
   */
  import type { MechanismSpan, MechanismTimeline } from "../../utils/mechanismStates";

  export let timeline: MechanismTimeline;
  export let percent: number;
  export let handleSeek: (percent: number) => void;

  $: total = timeline.total || 0;
  const pct = (t: number) => (total > 0 ? Math.max(0, Math.min(100, (t / total) * 100)) : 0);

  type Lane = { name: string; spans: (MechanismSpan & { color: string })[]; ticks: number[] };
  $: lanes = [
    {
      name: "Intake",
      spans: [
        ...timeline.intakeOn.map((s) => ({ ...s, color: "#22c55e" })),
        ...timeline.intakeFeed.map((s) => ({ ...s, color: "#22d3ee" })),
      ],
      ticks: [],
    },
    {
      name: "Scoopula",
      spans: [
        ...timeline.scoopulaDown.map((s) => ({ ...s, color: "#f97316" })),
        ...timeline.scoopulaMoving.map((s) => ({ ...s, color: "#fdba74" })),
      ],
      ticks: [],
    },
    { name: "Shoot", spans: timeline.shoot.map((s) => ({ ...s, color: "#ef4444" })), ticks: [] },
    {
      name: "Pick / turn",
      spans: [
        ...timeline.backoff.map((s) => ({ ...s, color: "#a78bfa" })),
        ...timeline.turn.map((s) => ({ ...s, color: "#7c3aed" })),
        ...timeline.otherEvents.map((s) => ({ ...s, color: "#9ca3af" })),
      ],
      ticks: timeline.picks,
    },
  ] as Lane[];

  const fmt = (t: number) => `${t.toFixed(1)}s`;
</script>

{#if total > 0}
  <div class="w-full bg-neutral-50 dark:bg-neutral-900 rounded-lg px-3 py-2 shadow-lg text-[11px]">
    {#each lanes as lane}
      <div class="flex flex-row items-center gap-2 h-[18px]">
        <div class="w-[72px] shrink-0 text-neutral-600 dark:text-neutral-300 font-semibold">{lane.name}</div>
        <div class="relative w-full h-[12px] rounded bg-neutral-200 dark:bg-neutral-800">
          {#each lane.spans as span}
            <div
              class="absolute top-0 h-full rounded cursor-pointer"
              role="button"
              tabindex="0"
              title={`${span.label}: ${fmt(span.start)} - ${fmt(span.end)} (click to jump)`}
              style={`left: ${pct(span.start)}%; width: ${Math.max(0.6, pct(span.end) - pct(span.start))}%; background: ${span.color};`}
              on:click={() => handleSeek(pct(span.start) + 0.001)}
              on:keydown={(e) => {
                if (e.key === "Enter" || e.key === " ") handleSeek(pct(span.start) + 0.001);
              }}
            ></div>
          {/each}
          {#each lane.ticks as t}
            <div
              class="absolute -top-[2px] h-[16px] w-[4px] -ml-[2px] rounded cursor-pointer"
              role="button"
              tabindex="0"
              title={`Camera pick at ${fmt(t)} (click to jump)`}
              style={`left: ${pct(t)}%; background: #facc15; box-shadow: 0 0 0 1px #000;`}
              on:click={() => handleSeek(pct(t) + 0.001)}
              on:keydown={(e) => {
                if (e.key === "Enter" || e.key === " ") handleSeek(pct(t) + 0.001);
              }}
            ></div>
          {/each}
          <div
            class="absolute -top-[3px] h-[18px] w-[2px] bg-black dark:bg-white pointer-events-none"
            style={`left: ${Math.max(0, Math.min(100, percent))}%;`}
          ></div>
        </div>
      </div>
    {/each}
    <div class="flex flex-row flex-wrap gap-x-3 gap-y-1 mt-1 text-neutral-500 dark:text-neutral-400">
      <span><span style="color:#22c55e">■</span> intake on</span>
      <span><span style="color:#22d3ee">■</span> feeding (Shoot: gate may be open)</span>
      <span><span style="color:#f97316">■</span> scoopula down</span>
      <span><span style="color:#fdba74">■</span> moving</span>
      <span><span style="color:#ef4444">■</span> shoot</span>
      <span><span style="color:#facc15">■</span> camera pick</span>
      <span><span style="color:#a78bfa">■</span> back off</span>
      <span><span style="color:#7c3aed">■</span> turn</span>
    </div>
  </div>
{/if}
