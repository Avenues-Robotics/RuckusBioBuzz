<script lang="ts">
  /**
   * The mechanisms drawn ON the robot at the playhead (utils/mechanismStates.ts):
   * intake bar along the FRONT edge (green on, cyan feeding the shooter, grey off),
   * scoopula tab off the front (orange when down, outline when up), the turret's aim
   * line while shooting, and a badge stack beside the robot.
   */
  import type { MechanismState } from "../../utils/mechanismStates";

  /** Robot centre, screen px. */
  export let x: number;
  export let y: number;
  /** CSS rotation of the robot sprite, degrees clockwise (= -field heading). */
  export let heading: number;
  export let lengthPx: number;
  export let widthPx: number;
  export let pxPerInch: number;
  export let state: MechanismState;
  /** Turret and aim point, screen px, while shooting. */
  export let turret: { x: number; y: number } | null = null;
  export let aim: { x: number; y: number } | null = null;

  $: half = lengthPx / 2;
  $: bar = Math.max(3, pxPerInch * 1.6);
  $: tabOut = pxPerInch * 4;
  $: tabWide = widthPx * 0.55;
  $: intakeColor = state.intake === "on" ? "#22c55e" : state.intake === "feed" ? "#22d3ee" : "#6b7280";
  $: scoopColor = "#f97316";
  $: scoopOut = state.scoopula === "down" ? tabOut : state.scoopula === "up" ? 0 : tabOut / 2;

  $: badges = [
    state.pickFlash ? { text: "CAMERA PICK", color: "#facc15" } : null,
    state.phase === "backoff" ? { text: "BACK OFF", color: "#a78bfa" } : null,
    state.phase === "turn" ? { text: "TURN", color: "#a78bfa" } : null,
    state.shooting ? { text: "SHOOT", color: "#ef4444" } : null,
    state.intake === "on"
      ? { text: "INTAKE ON", color: "#22c55e" }
      : state.intake === "feed"
        ? { text: "INTAKE FEEDING", color: "#22d3ee" }
        : { text: "INTAKE OFF", color: "#6b7280" },
    state.scoopula === "down"
      ? { text: "SCOOPULA DOWN", color: "#f97316" }
      : state.scoopula === "lowering"
        ? { text: "SCOOPULA LOWERING", color: "#f97316" }
        : state.scoopula === "raising"
          ? { text: "SCOOPULA RAISING", color: "#fdba74" }
          : null,
    ...state.events.map((name) => ({ text: name.toUpperCase(), color: "#e5e7eb" })),
  ].filter(Boolean) as { text: string; color: string }[];
</script>

<svg
  style={`position: absolute; top: ${y}px; left: ${x}px; z-index: 23; pointer-events: none; overflow: visible;`}
  width="1"
  height="1"
>
  {#if state.shooting && turret && aim}
    <line
      x1={turret.x - x}
      y1={turret.y - y}
      x2={aim.x - x}
      y2={aim.y - y}
      stroke="#ef4444"
      stroke-width="3"
      stroke-dasharray="8 5"
    />
    <circle cx={aim.x - x} cy={aim.y - y} r="6" fill="#ef4444" stroke="#fff" stroke-width="2" />
    <circle cx={turret.x - x} cy={turret.y - y} r="4" fill="#ef4444" stroke="#fff" stroke-width="1.5" />
  {/if}

  <g transform={`rotate(${heading})`}>
    <!-- Scoopula: a tab out of the FRONT; dashed outline when up. -->
    {#if scoopOut > 0}
      <rect
        x={half}
        y={-tabWide / 2}
        width={scoopOut}
        height={tabWide}
        fill={scoopColor}
        fill-opacity={state.scoopula === "down" ? 0.9 : 0.55}
        stroke="#7c2d12"
        stroke-width="1.5"
      />
    {:else}
      <rect
        x={half - pxPerInch * 1.2}
        y={-tabWide / 2}
        width={pxPerInch * 1.2}
        height={tabWide}
        fill="none"
        stroke={scoopColor}
        stroke-width="1.5"
        stroke-dasharray="3 2"
      />
    {/if}
    <!-- Intake: a bar along the FRONT edge. -->
    <rect
      x={half - bar}
      y={-widthPx / 2}
      width={bar}
      height={widthPx}
      fill={intakeColor}
      stroke="#000"
      stroke-width="1"
      opacity={state.intake === "off" ? 0.75 : 1}
    />
  </g>

  <g transform={`translate(${Math.max(lengthPx, widthPx) / 2 + 8}, ${-Math.max(lengthPx, widthPx) / 2})`}>
    {#each badges as badge, i}
      <g transform={`translate(0, ${i * 17})`}>
        <rect x="0" y="0" width={badge.text.length * 6.6 + 10} height="15" rx="3" fill="#111827" fill-opacity="0.85" stroke={badge.color} stroke-width="1.5" />
        <text x="5" y="11" font-size="10.5" font-weight="700" font-family="ui-sans-serif, system-ui, sans-serif" fill={badge.color}>{badge.text}</text>
      </g>
    {/each}
  </g>
</svg>
