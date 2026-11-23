<script lang="ts">
  import type {
    EventMarker,
    EventTriggerType,
    Line,
    PoseVariable,
    Variable,
  } from "../../types";
  import { snapToGrid, showGrid, gridSize } from "../../stores";
  import {
    canMoveEndpoint,
    clearanceTargetName,
    type ClearanceLineReport,
  } from "../../utils/clearance";
  import {
    expressionDisplayValue,
    pointCoordinateFieldDisplayValue,
    resolveEventMarkerExpressions,
    resolveLineExpressions,
    resolvePointExpressions,
  } from "../../utils";
  import ControlPointsSection from "./ControlPointsSection.svelte";
  import ExpressionInput from "./ExpressionInput.svelte";
  import { normalizeCameraPick } from "../../utils/cameraPick";
  import { FIELD_SIZE } from "../../config/defaults";
  import HeadingControls from "./HeadingControls.svelte";
  import HeadingCurveEditor from "./HeadingCurveEditor.svelte";
  import { scrubbable } from "../../utils/scrub";

  export let line: Line;
  export let idx: number;
  export let lines: Line[];
  export let collapsed: boolean;
  export let collapsedControlPoints: boolean;
  export let onRemove: () => void;
  export let onInsertAfter: () => void;
  export let onInsertMidpoint: () => void;
  export let onDuplicate: () => void = () => {};
  export let onWrapRepeat: () => void = () => {};
  export let onWrapConditional: () => void = () => {};
  /** Drag handle wiring: lets this path be dropped into a loop or if block. */
  export let onPointerDown: (event: PointerEvent) => void = () => {};
  export let dragging = false;
  export let onAddWaitAfter: () => void;
  export let onAddEventAfter: () => void;
  export let recordChange: () => void;
  export let onMoveUp: () => void;
  export let onMoveDown: () => void;
  export let canMoveUp: boolean = true;
  export let canMoveDown: boolean = true;
  export let optimizeLine: (lineId: string, targetControlPointIndex?: number) => void;
  export let optimizing: boolean = false;
  export let poseVariables: PoseVariable[] = [];
  export let variables: Variable[] = [];
  export let onPoseVariableChange: (lineId: string, poseVariableId: string) => void = () => {};
  export let onHeadingModeChange: (lineId: string, mode: string) => void = () => {};
  /** True when this path is skipped by its own or its chain's condition. */
  export let inactive = false;
  /**
   * Why the robot comes to a stop at the end of this path, or null when it
   * carries straight on into the next one.
   */
  export let stopReason: string | null = null;
  /**
   * Degrees between the heading the robot arrives with and the heading this
   * path's interpolation asks for. The robot turns to make it up while driving,
   * so a large jump means it is fighting its heading mid-path.
   */
  export let headingJump: number | null = null;
  export let onMatchEntryHeading: () => void = () => {};

  /**
   * How close the robot's body comes to an obstacle or a wall on this path.
   * Null when nothing on the path is inside the safety margin.
   */
  export let clearance: ClearanceLineReport | null = null;
  /** Moves this path's endpoint on X and Y until the robot clears. */
  export let onFixClearance: () => void = () => {};
  export let fixingClearance = false;
  /** What the last fix on this path achieved, or why it could not. */
  export let clearanceFixNote = "";

  // A locked endpoint was pinned on purpose, and one driven by a pose variable
  // takes its position from the variable, so writing x/y here would not stick.
  $: canFixClearance = canMoveEndpoint(line);

  $: headingJumpDegrees = Math.abs(Math.round(Number(headingJump) || 0));
  $: showHeadingJump = headingJumpDegrees >= 2;
  $: canMatchEntryHeading = line.endPoint.heading === "linear";

  $: worstClearanceSpan = clearance?.spans.length
    ? clearance.spans.reduce((worst, span) =>
        span.worstClearance < worst.worstClearance ? span : worst,
      )
    : null;

  $: clearanceLabel = !worstClearanceSpan
    ? ""
    : worstClearanceSpan.severity === "hit"
      ? worstClearanceSpan.kind === "midline"
        ? "Crosses the midline"
        : `Hits ${clearanceTarget}`
      : `${worstClearanceSpan.worstClearance.toFixed(1)}in clearance`;

  $: clearanceTarget = !worstClearanceSpan
    ? ""
    : clearanceTargetName(worstClearanceSpan);

  $: clearanceTitle = !worstClearanceSpan
    ? ""
    : worstClearanceSpan.severity === "hit"
      ? worstClearanceSpan.kind === "midline"
        ? `The robot crosses the midline by ${Math.abs(
            worstClearanceSpan.worstClearance,
          ).toFixed(2)}in at its furthest, first crossing ${(
            worstClearanceSpan.contactDistance ?? worstClearanceSpan.startDistance
          ).toFixed(1)}in into this path. AUTO columns A-C are the red side and D-F the blue (G402).`
        : `The robot overlaps the ${clearanceTarget} by ${Math.abs(
            worstClearanceSpan.worstClearance,
          ).toFixed(2)}in, first touching ${(
            worstClearanceSpan.contactDistance ?? worstClearanceSpan.startDistance
          ).toFixed(1)}in into this path.`
      : `The robot passes within ${worstClearanceSpan.worstClearance.toFixed(
          2,
        )}in of the ${clearanceTarget}, ${worstClearanceSpan.startDistance.toFixed(
          1,
        )}in into this path. Checked against the robot's footprint at the heading it actually holds.`;

  function commitLine() {
    lines = lines.map((item) => (item.id === line.id ? line : item));
    recordChange?.();
  }

  function handleTurnFirstToggle(event: Event) {
    line.turnFirst = (event.currentTarget as HTMLInputElement).checked || undefined;
    commitLine();
  }

  /** Camera pick: on with the defaults for whichever half of the field this path ends on. */
  function handleCameraPickToggle(event: Event) {
    const on = (event.currentTarget as HTMLInputElement).checked;
    if (on) {
      const blue = Number(line.endPoint?.y) > FIELD_SIZE / 2;
      line.cameraPick = normalizeCameraPick(
        { ...(line.cameraPick || {}), enabled: true, ...(line.cameraPick ? {} : { alliance: blue ? "BLUE" : "RED" }) },
        line.endPoint,
      );
    } else if (line.cameraPick) {
      line.cameraPick = { ...line.cameraPick, enabled: false };
    }
    commitLine();
  }

  function setCameraPickNumber(key: "pickY" | "wallY" | "xMin" | "xMax" | "previewX" | "hfovDeg" | "cameraForward", raw: string) {
    const value = Number(raw);
    if (!line.cameraPick || !Number.isFinite(value)) return;
    line.cameraPick = normalizeCameraPick({ ...line.cameraPick, [key]: value }, line.endPoint);
    commitLine();
  }

  const cameraPickFields: { key: "pickY" | "wallY" | "xMin" | "xMax" | "previewX" | "hfovDeg" | "cameraForward"; label: string; title: string }[] = [
    { key: "previewX", label: "Clump x", title: "Editor only: where the clump sits along the wall. Drag the yellow clump on the field." },
    { key: "pickY", label: "Pick y", title: "The frame is read where this path passes (path end x, Pick y) - AutoConstants PICK_Y (mirrored for BLUE)." },
    { key: "wallY", label: "Wall y", title: "Robot centre against the wall - AutoConstants WALL_Y (mirrored for BLUE)." },
    { key: "xMin", label: "x min", title: "Allowed drive-to x along the wall - AutoConstants PICK_X_MIN (mirrored for BLUE)." },
    { key: "xMax", label: "x max", title: "Allowed drive-to x along the wall - AutoConstants PICK_X_MAX (mirrored for BLUE)." },
    { key: "hfovDeg", label: "FOV°", title: "Horizontal field of view of the upright frame - SystemConstants.Camera.HFOV_DEG (estimated)." },
    { key: "cameraForward", label: "Lens fwd", title: "Lens ahead of the robot centre, inches - SystemConstants.Camera.CAMERA_OFFSET[0] (estimated)." },
  ];

  function handleStopAtEndToggle(event: Event) {
    line.stopAtEnd = (event.currentTarget as HTMLInputElement).checked;
    lines = lines.map((item) => (item.id === line.id ? line : item));
    recordChange?.();
  }


  $: snapToGridTitle =
    $snapToGrid && $showGrid ? `Snapping to ${$gridSize} grid` : "No snapping";
  $: isEndPointBoundToPoseVariable = Boolean(line.endPoint?.poseVariableId);
  $: boundPoseVariableHeading = (() => {
    const poseVariableId = line.endPoint?.poseVariableId;
    if (!poseVariableId) return null;
    const variable = poseVariables.find((item) => item.id === poseVariableId);
    if (!variable) return null;
    const heading = Number(variable.heading);
    return Number.isFinite(heading) ? heading : 0;
  })();
  $: pathSpeedValue = clampPathSpeed(line?.speed);
  $: eventMarkers = line.eventMarkers || [];
  $: hasSpeedExpression = Boolean(line.speedExpression?.trim());

  function toggleCollapsed() {
    collapsed = !collapsed;
  }

  function clampPathSpeed(value: number | undefined): number {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) return 1;
    return Math.max(0.05, Math.min(1, numeric));
  }

  function handlePathSpeedInput(event: Event) {
    const target = event.currentTarget as HTMLInputElement;
    line.speed = clampPathSpeed(Number(target.value));
    lines = [...lines];
  }

  /** Writes an expression onto the line and refreshes its resolved literals. */
  function setLineExpression(
    field: "speedExpression" | "enabledExpression",
    raw: string,
  ) {
    line = resolveLineExpressions(
      { ...line, [field]: raw.trim() ? raw : undefined },
      variables,
    );
    lines = lines.map((item) => (item.id === line.id ? line : item));
  }

  function commitPathSpeed() {
    line.speed = clampPathSpeed(line.speed);
    lines = [...lines];
    recordChange?.();
  }

  function handleColorInput(event: Event) {
    line.color = (event.currentTarget as HTMLInputElement).value;
    lines = [...lines];
  }

  function handlePoseVariableSelect(event: Event) {
    onPoseVariableChange(line.id || "", (event.currentTarget as HTMLSelectElement).value);
  }

  function updateEndPointCoordinate(field: "x" | "y", value: string) {
    const expressionField = `${field}Expression` as "xExpression" | "yExpression";
    const numeric = Number(value);
    line.endPoint = resolvePointExpressions(
      {
        ...line.endPoint,
        [field]: Number.isFinite(numeric) ? numeric : line.endPoint[field],
        [expressionField]: value.trim() ? value : undefined,
      },
      variables,
    );
    lines = [...lines];
  }

  function makeEventMarkerId() {
    return `event-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
  }

  function clampEventPosition(value: number | undefined): number {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) return 0.5;
    return Math.max(0, Math.min(1, numeric));
  }

  function clampEventDuration(value: number | undefined): number {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) return 0;
    return Math.max(0, Math.round(numeric));
  }

  function eventTriggerType(marker: EventMarker): EventTriggerType {
    return marker.triggerType === "temporal" || marker.triggerType === "pose"
      ? marker.triggerType
      : "parametric";
  }

  function eventPoseX(marker: EventMarker): number {
    const numeric = Number(marker.poseX);
    return Number.isFinite(numeric) ? numeric : Number(line.endPoint?.x ?? 0);
  }

  function eventPoseY(marker: EventMarker): number {
    const numeric = Number(marker.poseY);
    return Number.isFinite(numeric) ? numeric : Number(line.endPoint?.y ?? 0);
  }

  function normalizeEventMarker(marker: EventMarker, index: number): EventMarker {
    const triggerMs = Number(marker.triggerMs ?? 0);
    return {
      ...marker,
      id: marker.id || makeEventMarkerId(),
      name: marker.name || `Event ${index + 1}`,
      triggerType: eventTriggerType(marker),
      position: clampEventPosition(marker.position),
      triggerMs: Number.isFinite(triggerMs)
        ? Math.max(0, Math.round(triggerMs))
        : 0,
      poseX: eventPoseX(marker),
      poseY: eventPoseY(marker),
      durationMs: clampEventDuration(marker.durationMs),
    };
  }

  function setEventMarkers(nextMarkers: EventMarker[], commit = false) {
    line.eventMarkers = nextMarkers.map(normalizeEventMarker);
    lines = [...lines];
    if (commit) recordChange?.();
  }

  function addEventMarker() {
    const nextIndex = eventMarkers.length;
    setEventMarkers(
      [
        ...eventMarkers,
        {
          id: makeEventMarkerId(),
          name: `Event ${nextIndex + 1}`,
          triggerType: "parametric",
          position: 0.5,
          triggerMs: 0,
          poseX: Number(line.endPoint?.x ?? 0),
          poseY: Number(line.endPoint?.y ?? 0),
          durationMs: 0,
        },
      ],
      true,
    );
  }

  function updateEventMarker(index: number, patch: Partial<EventMarker>, commit = false) {
    const nextMarkers = eventMarkers.map((marker, markerIndex) =>
      markerIndex === index ? { ...marker, ...patch } : marker,
    );
    setEventMarkers(nextMarkers, commit);
  }

  function handleEventNameInput(index: number, event: Event) {
    updateEventMarker(index, {
      name: (event.currentTarget as HTMLInputElement).value,
    });
  }

  function handleEventPositionInput(index: number, event: Event) {
    updateEventMarker(index, {
      position: Number((event.currentTarget as HTMLInputElement).value),
    });
  }

  type EventExpressionField =
    | "positionExpression"
    | "triggerMsExpression"
    | "poseXExpression"
    | "poseYExpression"
    | "durationExpression"
    | "enabledExpression";

  /** Sets an event-marker expression and refreshes the literal it resolves to. */
  function setEventExpression(
    index: number,
    field: EventExpressionField,
    raw: string,
  ) {
    const marker = eventMarkers[index];
    if (!marker) return;

    const next = { ...marker, [field]: raw.trim() ? raw : undefined };
    const nextMarkers = eventMarkers.map((item, markerIndex) =>
      markerIndex === index
        ? field === "enabledExpression"
          ? next
          : resolveEventMarkerExpressions(next, variables)
        : item,
    );

    line.eventMarkers = nextMarkers;
    lines = [...lines];
  }

  function handleEventTriggerTypeInput(index: number, event: Event) {
    const triggerType = (event.currentTarget as HTMLSelectElement).value as EventTriggerType;
    updateEventMarker(
      index,
      {
        triggerType,
        poseX: eventPoseX(eventMarkers[index]),
        poseY: eventPoseY(eventMarkers[index]),
      },
      true,
    );
  }

  function removeEventMarker(index: number) {
    setEventMarkers(
      eventMarkers.filter((_, markerIndex) => markerIndex !== index),
      true,
    );
  }

</script>

<div
  class="flex flex-col w-full justify-start items-start gap-1 rounded-md p-1 {inactive
    ? 'opacity-60'
    : ''} {dragging ? 'opacity-50 ring-2 ring-sky-400/70' : ''}"
>
  <div class="flex flex-row w-full items-center gap-3 flex-wrap">
    <div class="flex flex-row items-center gap-2">
      <!-- Drag handle: drop this path into any repeat loop or if block -->
      <span
        role="button"
        tabindex={line.locked ? -1 : 0}
        draggable="false"
        on:pointerdown={onPointerDown}
        on:dragstart|preventDefault
        title={line.locked
          ? "Locked paths cannot be dragged"
          : "Drag into a repeat loop or if block"}
        aria-label="Drag path"
        class="shrink-0 select-none touch-none rounded px-1 py-0.5 text-neutral-400 hover:text-neutral-700 dark:hover:text-neutral-200 hover:bg-neutral-200/70 dark:hover:bg-neutral-800 {line.locked
          ? 'cursor-not-allowed opacity-40'
          : dragging
            ? 'cursor-grabbing'
            : 'cursor-grab'}"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="currentColor"
          class="size-4 pointer-events-none"
        >
          <circle cx="9" cy="6" r="1.6" />
          <circle cx="15" cy="6" r="1.6" />
          <circle cx="9" cy="12" r="1.6" />
          <circle cx="15" cy="12" r="1.6" />
          <circle cx="9" cy="18" r="1.6" />
          <circle cx="15" cy="18" r="1.6" />
        </svg>
      </span>

      <button
        on:click={toggleCollapsed}
        class="flex items-center gap-2 font-semibold px-2 py-1 rounded transition-colors duration-250"
        title="{collapsed ? 'Expand' : 'Collapse'} path"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          stroke-width={2}
          stroke="currentColor"
          class="size-4 transition-transform {collapsed
            ? 'rotate-0'
            : 'rotate-90'}"
        >
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="m8.25 4.5 7.5 7.5-7.5 7.5"
          />
        </svg>
        Path {idx + 1}
      </button>

      {#if inactive}
        <span
          class="rounded-full bg-neutral-200 dark:bg-neutral-800 px-2 py-0.5 text-[10px] font-semibold text-neutral-600 dark:text-neutral-300"
          title="This path's own Enabled-if condition is currently false, so it is skipped"
        >
          Skipped
        </span>
      {/if}

      <!--
        A heading goal that does not pick up where the previous path left off
        makes the robot turn while it drives. On a linear path the chip fixes
        it; otherwise the geometry decides and it is just a warning.
      -->
      {#if showHeadingJump}
        {#if canMatchEntryHeading}
          <button
            on:click|stopPropagation={onMatchEntryHeading}
            class="rounded-full bg-rose-100 dark:bg-rose-900/60 px-2 py-0.5 text-[10px] font-semibold text-rose-700 dark:text-rose-200 hover:bg-rose-200 dark:hover:bg-rose-900"
            title="This path starts {headingJumpDegrees}° away from the heading the robot arrives with, so it turns while driving. Click to start it at the incoming heading instead."
          >
            Heading jump {headingJumpDegrees}° — fix
          </button>
        {:else}
          <span
            class="rounded-full bg-rose-100 dark:bg-rose-900/60 px-2 py-0.5 text-[10px] font-semibold text-rose-700 dark:text-rose-200"
            title="This path's heading starts {headingJumpDegrees}° away from the heading the robot arrives with, so it turns while driving."
          >
            Heading jump {headingJumpDegrees}°
          </span>
        {/if}
      {/if}

      <!--
        The robot's footprint, not the path, is what has to fit. A curve can
        thread a gap the robot cannot, so this says when the body is in trouble
        even though the line through the middle of it looks fine.
      -->
      {#if worstClearanceSpan}
        {#if canFixClearance}
          <button
            on:click|stopPropagation={onFixClearance}
            disabled={fixingClearance}
            class="rounded-full px-2 py-0.5 text-[10px] font-semibold disabled:opacity-60 {worstClearanceSpan.severity ===
            'hit'
              ? 'bg-red-100 dark:bg-red-900/60 text-red-700 dark:text-red-200 hover:bg-red-200 dark:hover:bg-red-900'
              : 'bg-amber-100 dark:bg-amber-900/60 text-amber-700 dark:text-amber-200 hover:bg-amber-200 dark:hover:bg-amber-900'}"
            title="{clearanceTitle} Click to move this path's endpoint on X and Y until the robot clears."
          >
            {fixingClearance ? "Fixing…" : `${clearanceLabel} — fix collision issues`}
          </button>
        {:else}
          <span
            class="rounded-full px-2 py-0.5 text-[10px] font-semibold {worstClearanceSpan.severity ===
            'hit'
              ? 'bg-red-100 dark:bg-red-900/60 text-red-700 dark:text-red-200'
              : 'bg-amber-100 dark:bg-amber-900/60 text-amber-700 dark:text-amber-200'}"
            title={clearanceTitle}
          >
            {clearanceLabel}
          </span>
        {/if}
      {/if}

      <!-- What the endpoint move actually achieved, or why it could not. -->
      {#if clearanceFixNote}
        <span
          class="rounded-full bg-neutral-200 dark:bg-neutral-800 px-2 py-0.5 text-[10px] font-semibold text-neutral-600 dark:text-neutral-300"
          title={clearanceFixNote}
        >
          {clearanceFixNote}
        </span>
      {/if}

      <!--
        Stops are what cost time now that consecutive paths are chained, so the
        row says where one happens without having to expand the path.
      -->
      {#if stopReason}
        <span
          class="rounded-full bg-amber-100 dark:bg-amber-900/60 px-2 py-0.5 text-[10px] font-semibold text-amber-700 dark:text-amber-200"
          title="The robot decelerates to a stop at this endpoint because {stopReason}."
        >
          Stops
        </span>
      {:else}
        <span
          class="rounded-full bg-emerald-50 dark:bg-emerald-900/40 px-2 py-0.5 text-[10px] font-semibold text-emerald-700 dark:text-emerald-200"
          title="The robot drives straight through this endpoint into the next path — they are followed as one PathChain."
        >
          Chains on
        </span>
      {/if}

      <input
        bind:value={line.name}
        placeholder="Path {idx + 1}"
        class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-950 dark:border-neutral-700 border-[0.5px] focus:outline-none text-sm font-semibold"
        disabled={line.locked}
        on:input={() => {
          // Force parent reactivity so other components (like exporters)
          // pick up the updated name immediately.
          lines = [...lines];
        }}
        on:blur={() => {
          // Commit the change for history/undo
          lines = [...lines];
          if (recordChange) recordChange();
        }}
      />

      <input
        type="color"
        value={line.color}
        on:input={handleColorInput}
        on:change={() => recordChange?.()}
        disabled={line.locked}
        class="size-6 rounded border border-neutral-300 dark:border-neutral-600 bg-transparent shrink-0 disabled:opacity-40"
        title="Path color"
      />

      <!-- Lock/Unlock Button -->
      <button
        title={line.locked ? "Unlock Path" : "Lock Path"}
        on:click|stopPropagation={() => {
          line.locked = !line.locked;
          lines = [...lines]; // Force reactivity
        }}
        class="p-1 rounded transition-colors duration-250"
      >
        {#if line.locked}
          <svg
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
            stroke-width={2}
            stroke="currentColor"
            class="size-5 stroke-yellow-500"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="M16.5 10.5V6.75a4.5 4.5 0 1 0-9 0v3.75m-.75 11.25h10.5a2.25 2.25 0 0 0 2.25-2.25v-6.75a2.25 2.25 0 0 0-2.25-2.25H6.75a2.25 2.25 0 0 0-2.25 2.25v6.75a2.25 2.25 0 0 0 2.25 2.25Z"
            />
          </svg>
        {:else}
          <svg
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
            stroke-width={2}
            stroke="currentColor"
            class="size-5 stroke-gray-400"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="M13.5 10.5V6.75a4.5 4.5 0 1 1 9 0v3.75M3.75 21.75h10.5a2.25 2.25 0 0 0 2.25-2.25v-6.75a2.25 2.25 0 0 0-2.25-2.25H3.75a2.25 2.25 0 0 0-2.25 2.25v6.75a2.25 2.25 0 0 0 2.25 2.25Z"
            />
          </svg>
        {/if}
      </button>

      <div class="flex flex-row gap-0.5 ml-1">
        <button
          title={line.locked ? "Path locked" : "Move up"}
          on:click|stopPropagation={() => {
            if (!line.locked && onMoveUp) onMoveUp();
          }}
          class="p-1 rounded-full text-neutral-500 dark:text-neutral-400 hover:text-neutral-700 dark:hover:text-neutral-200 bg-neutral-100/70 dark:bg-neutral-900/70 border border-neutral-200/70 dark:border-neutral-700/70 disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={!canMoveUp || line.locked}
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            class="size-4"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="m5 15 7-7 7 7"
            />
          </svg>
        </button>
        <button
          title={line.locked ? "Path locked" : "Move down"}
          on:click|stopPropagation={() => {
            if (!line.locked && onMoveDown) onMoveDown();
          }}
          class="p-1 rounded-full text-neutral-500 dark:text-neutral-400 hover:text-neutral-700 dark:hover:text-neutral-200 bg-neutral-100/70 dark:bg-neutral-900/70 border border-neutral-200/70 dark:border-neutral-700/70 disabled:opacity-40 disabled:cursor-not-allowed"
          disabled={!canMoveDown || line.locked}
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            class="size-4"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="m19 9-7 7-7-7"
            />
          </svg>
        </button>
      </div>
    </div>

    <div class="flex flex-row items-center gap-1">
      <button
        class="px-2 py-1 text-xs font-semibold text-neutral-700 dark:text-neutral-200 bg-neutral-200/80 dark:bg-neutral-800/80 border border-neutral-300 dark:border-neutral-700 rounded disabled:opacity-40 disabled:cursor-not-allowed"
        title={line.locked ? "Path locked" : "Optimize this path"}
        on:click={() => line.id && optimizeLine && optimizeLine(line.id)}
        disabled={!line.id || line.locked || optimizing}
      >
        {optimizing ? "Optimizing…" : "Optimize"}
      </button>
      <button
        class="px-2 py-1 text-xs font-semibold text-neutral-700 dark:text-neutral-200 bg-neutral-200/80 dark:bg-neutral-800/80 border border-neutral-300 dark:border-neutral-700 rounded disabled:opacity-40 disabled:cursor-not-allowed"
        title={line.locked ? "Path locked" : "Duplicate this path"}
        on:click={onDuplicate}
        disabled={line.locked}
      >
        Duplicate
      </button>
      <button
        class="px-2 py-1 text-xs font-semibold text-neutral-700 dark:text-neutral-200 bg-neutral-200/80 dark:bg-neutral-800/80 border border-neutral-300 dark:border-neutral-700 rounded disabled:opacity-40 disabled:cursor-not-allowed"
        title={line.locked ? "Path locked" : "Wrap this path in a repeat loop"}
        on:click={onWrapRepeat}
        disabled={line.locked}
      >
        Loop
      </button>
      <button
        class="px-2 py-1 text-xs font-semibold font-mono text-violet-700 dark:text-violet-200 bg-violet-100 dark:bg-violet-900/60 border border-violet-300 dark:border-violet-800 rounded disabled:opacity-40 disabled:cursor-not-allowed"
        title={line.locked ? "Path locked" : "Wrap this path in an if block"}
        on:click={onWrapConditional}
        disabled={line.locked}
      >
        if
      </button>
    </div>

    <div class="flex flex-row items-center gap-1 ml-auto">
      <button
        title="Add control point after this line"
        on:click={onInsertAfter}
        class="text-blue-500 hover:text-blue-600 disabled:opacity-40 disabled:cursor-not-allowed"
        disabled={line.locked}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          stroke-width={2}
          class="size-5 stroke-green-500"
        >
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 4.5v15m7.5-7.5h-15"
          />
        </svg>
      </button>

      <!-- Insert Midpoint Between This and Next Path (dark-blue plus icon) -->
      <button
        title="Insert point between this path and the next"
        on:click={() => onInsertMidpoint && onInsertMidpoint()}
        class="text-blue-700 hover:text-blue-500"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          fill="none"
          viewBox="0 0 24 24"
          stroke-width={2}
          stroke="currentColor"
          class="size-5"
        >
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M5 8h4m6 0h4m-9 0 1.75-2.5M12 6l1.25 2.5"
          />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M5 16h4m6 0h4m-9 0 1.75 2.5M12 18l1.25-2.5"
          />
          <circle cx="12" cy="12" r="2.1" />
        </svg>
      </button>

      <!-- Add Wait After Button -->
      <button
        title="Add Wait After"
        on:click={onAddWaitAfter}
        class="text-[#E1461B] hover:text-orange-600"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          stroke-width="2"
          class="size-5"
        >
          <circle cx="12" cy="12" r="9" />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 7v5l3 2"
          />
        </svg>
      </button>

      <button
        title="Add shoot event after"
        on:click={onAddEventAfter}
        class="text-purple-500 hover:text-purple-600"
        disabled={line.locked}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          stroke-width="2"
          class="size-5"
        >
          <circle cx="12" cy="12" r="8" />
          <circle cx="12" cy="12" r="2" />
          <path
            stroke-linecap="round"
            stroke-linejoin="round"
            d="M12 2v4M12 18v4M2 12h4M18 12h4"
          />
        </svg>
      </button>

      {#if lines.length > 1}
        <button title="Remove Line" on:click={onRemove}>
          <svg
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
            stroke-width={2}
            class="size-5 stroke-red-500"
          >
            <path
              stroke-linecap="round"
              stroke-linejoin="round"
              d="M15 12H9m12 0a9 9 0 1 1-18 0 9 9 0 0 1 18 0Z"
            />
          </svg>
        </button>
      {/if}
    </div>
  </div>

  {#if !collapsed}
    <div class={`h-[0.75px] w-full`} style={`background: ${line.color}`} />

    <div class="flex flex-col justify-start items-start w-full">
      <div class="font-light">Path Speed:</div>
      <div class="flex flex-row justify-start items-center gap-2 mb-2 w-full max-w-md">
        <input
          type="range"
          min="0.05"
          max="1"
          step="0.05"
          value={pathSpeedValue}
          on:input={handlePathSpeedInput}
          on:change={commitPathSpeed}
          disabled={line.locked || hasSpeedExpression}
          class="flex-1"
        />
        <div class="w-32">
          <ExpressionInput
            compact
            {variables}
            disabled={line.locked}
            placeholder="1"
            title="Path speed 0.05–1 (literal or expression)"
            value={expressionDisplayValue(line.speedExpression, pathSpeedValue)}
            onInput={(raw) => setLineExpression("speedExpression", raw)}
            onCommit={() => recordChange?.()}
          />
        </div>
      </div>

      <label
        class="flex flex-row items-center gap-2 mb-2 cursor-pointer select-none"
        title="Off: this path is merged into one PathChain with the paths around it and driven without stopping. On: the robot decelerates and settles on this endpoint before the next path."
      >
        <input
          type="checkbox"
          checked={Boolean(line.stopAtEnd)}
          disabled={line.locked}
          on:change={handleStopAtEndToggle}
        />
        <span class="font-light">Stop at end</span>
        {#if stopReason && !line.stopAtEnd}
          <span class="text-xs text-neutral-500 dark:text-neutral-400">
            — stops here anyway: {stopReason}
          </span>
        {/if}
      </label>

      <label
        class="flex flex-row items-center gap-2 mb-2 cursor-pointer select-none"
        title="As soon as the previous path ends - before any wait or event (e.g. a Shoot) in between - the robot backs straight off any wall until its corners clear it, turns in place to this path's end heading, then holds it for this path (TeamCode runTurnStep + TurnInPlace; back-off within 0.5 in or 0.8 s, turn within 4 deg or 1.2 s)."
      >
        <input
          type="checkbox"
          checked={Boolean(line.turnFirst)}
          disabled={line.locked}
          on:change={handleTurnFirstToggle}
        />
        <span class="font-light">Turn first</span>
        {#if line.turnFirst}
          <span class="text-xs text-neutral-500 dark:text-neutral-400">
            — right after the previous path: backs off any wall, turns in place to {Math.round(Number(line.endPoint.heading === "linear" ? line.endPoint.endDeg : line.endPoint.heading === "constant" ? line.endPoint.degrees : 0))}°, then any waits / events, then drives
          </span>
        {/if}
      </label>

      <label
        class="flex flex-row items-center gap-2 mb-1 cursor-pointer select-none"
        title="Partway down this path the robot reads one camera frame, picks the biggest pollen clump against the end wall and is re-routed into it (TeamCode auto/modules/CameraPick). Drag the clump on the field to see the route change."
      >
        <input
          type="checkbox"
          checked={Boolean(line.cameraPick?.enabled)}
          disabled={line.locked}
          on:change={handleCameraPickToggle}
        />
        <span class="font-light">Camera pick</span>
        {#if line.cameraPick?.enabled}
          <span class="text-xs text-neutral-500 dark:text-neutral-400">
            — {line.cameraPick.alliance}: drives into the clump at x {Math.max(line.cameraPick.xMin, Math.min(line.cameraPick.xMax, line.cameraPick.previewX)).toFixed(1)}{line.cameraPick.previewX < line.cameraPick.xMin || line.cameraPick.previewX > line.cameraPick.xMax ? " (clamped)" : ""}
          </span>
        {/if}
      </label>
      {#if line.cameraPick?.enabled}
        <div class="flex flex-row flex-wrap items-center gap-x-3 gap-y-1 mb-2 ml-6 text-xs">
          {#each cameraPickFields as field}
            <label class="flex flex-row items-center gap-1" title={field.title}>
              <span class="font-light">{field.label}</span>
              <input
                type="number"
                step="0.5"
                class="w-16 px-1 py-0.5 rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-100 dark:bg-neutral-900"
                disabled={line.locked}
                value={line.cameraPick[field.key]}
                on:change={(event) => setCameraPickNumber(field.key, event.currentTarget.value)}
              />
            </label>
          {/each}
        </div>
      {/if}

      <div class="flex flex-row justify-start items-center gap-2 mb-2 w-full max-w-md">
        <div class="font-light shrink-0">Enabled if:</div>
        <ExpressionInput
          compact
          kind="boolean"
          {variables}
          disabled={line.locked}
          placeholder="always"
          title="Boolean expression; when false this path is skipped in generated code"
          value={line.enabledExpression || ""}
          onInput={(raw) => setLineExpression("enabledExpression", raw)}
          onCommit={() => recordChange?.()}
        />
      </div>

      <div class="font-light">Point Position:</div>
      <div class="flex flex-row justify-start items-center gap-2 flex-wrap">
        <div class="font-extralight">Pose:</div>
        <select
          value={line.endPoint.poseVariableId || ""}
          on:change={handlePoseVariableSelect}
          class="px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-100 dark:bg-neutral-900"
          disabled={line.locked}
        >
          <option value="">Custom</option>
          {#each poseVariables as variable (variable.id)}
            <option value={variable.id}>{variable.name || "Unnamed Pose"}</option>
          {/each}
        </select>

        <div
          class="font-extralight"
          use:scrubbable={{
            value: pointCoordinateFieldDisplayValue(line.endPoint, "x"),
            disabled: line.locked || isEndPointBoundToPoseVariable,
            onInput: (raw) => updateEndPointCoordinate("x", raw),
            onCommit: () => recordChange?.(),
          }}
        >X:</div>
        <div class="w-28">
          <ExpressionInput
            {variables}
            compact
            title={snapToGridTitle}
            disabled={line.locked || isEndPointBoundToPoseVariable}
            value={pointCoordinateFieldDisplayValue(line.endPoint, "x")}
            onInput={(raw) => updateEndPointCoordinate("x", raw)}
            onCommit={() => recordChange?.()}
          />
        </div>
        <div
          class="font-extralight"
          use:scrubbable={{
            value: pointCoordinateFieldDisplayValue(line.endPoint, "y"),
            disabled: line.locked || isEndPointBoundToPoseVariable,
            onInput: (raw) => updateEndPointCoordinate("y", raw),
            onCommit: () => recordChange?.(),
          }}
        >Y:</div>
        <div class="w-28">
          <ExpressionInput
            {variables}
            compact
            title={snapToGridTitle}
            disabled={line.locked || isEndPointBoundToPoseVariable}
            value={pointCoordinateFieldDisplayValue(line.endPoint, "y")}
            onInput={(raw) => updateEndPointCoordinate("y", raw)}
            onCommit={() => recordChange?.()}
          />
        </div>

        <!--
          `bind:` is required: HeadingControls replaces the whole point object,
          and without the binding those edits would be dropped on the floor and
          then silently reverted on the next parent update.
        -->
        <HeadingControls
          bind:endPoint={line.endPoint}
          {variables}
          locked={line.locked}
          boundEndHeading={boundPoseVariableHeading}
          onHeadingModeChange={(mode) => onHeadingModeChange(line.id || "", mode)}
          on:change={() => {
            // Force reactivity so timeline recalculates immediately
            lines = lines.map((item) => (item.id === line.id ? line : item));
          }}
          on:commit={() => {
            // Commit change to history
            lines = lines.map((item) => (item.id === line.id ? line : item));
            recordChange();
          }}
        />
      </div>

      {#if line.endPoint.heading === "linear"}
        <div class="mt-2 w-full">
          <HeadingCurveEditor
            bind:endPoint={line.endPoint}
            locked={line.locked}
            {variables}
            on:change={() => {
              lines = lines.map((item) => (item.id === line.id ? line : item));
            }}
            on:commit={() => {
              lines = lines.map((item) => (item.id === line.id ? line : item));
              recordChange();
            }}
          />
        </div>
      {/if}

      <div class="mt-3 w-full border-t border-neutral-200 dark:border-neutral-800 pt-2">
        <div class="flex flex-row items-center gap-2 mb-2">
          <div class="font-light">Parallel Events:</div>
          <button
            title={line.locked ? "Path locked" : "Add parallel event"}
            on:click={addEventMarker}
            disabled={line.locked}
            class="p-1 rounded text-purple-600 hover:text-purple-700 disabled:opacity-40 disabled:cursor-not-allowed"
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              stroke-width="2"
              class="size-5"
            >
              <path stroke-linecap="round" stroke-linejoin="round" d="M12 5v14M5 12h14" />
            </svg>
          </button>
        </div>

        {#if eventMarkers.length > 0}
          <div class="flex flex-col gap-2 w-full">
            {#each eventMarkers as marker, eventIdx (marker.id || `${line.id}-event-${eventIdx}`)}
              <div
                class="flex flex-row items-center gap-2 flex-wrap w-full rounded border border-neutral-200 dark:border-neutral-800 bg-neutral-50 dark:bg-neutral-950 p-2"
              >
                <div class="font-extralight">Name:</div>
                <input
                  class="pl-1.5 rounded-md bg-neutral-100 dark:bg-neutral-900 dark:border-neutral-700 border-[0.5px] focus:outline-none w-36"
                  value={marker.name}
                  disabled={line.locked}
                  on:input={(event) => handleEventNameInput(eventIdx, event)}
                  on:blur={() => updateEventMarker(eventIdx, {}, true)}
                />

                <div class="font-extralight">Type:</div>
                <select
                  value={eventTriggerType(marker)}
                  disabled={line.locked}
                  on:change={(event) => handleEventTriggerTypeInput(eventIdx, event)}
                  class="px-2 py-1 text-xs rounded border border-neutral-300 dark:border-neutral-600 bg-neutral-100 dark:bg-neutral-900"
                >
                  <option value="parametric">Path %</option>
                  <option value="temporal">Time</option>
                  <option value="pose">Pose</option>
                </select>

                {#if eventTriggerType(marker) === "temporal"}
                  <div class="font-extralight">After ms:</div>
                  <div class="w-28">
                    <ExpressionInput
                      compact
                      {variables}
                      disabled={line.locked}
                      placeholder="0"
                      title="Trigger time in ms (literal or expression)"
                      value={expressionDisplayValue(
                        marker.triggerMsExpression,
                        clampEventDuration(marker.triggerMs),
                      )}
                      onInput={(raw) =>
                        setEventExpression(eventIdx, "triggerMsExpression", raw)}
                      onCommit={() => updateEventMarker(eventIdx, {}, true)}
                    />
                  </div>
                {:else}
                  <div class="font-extralight">
                    {eventTriggerType(marker) === "pose" ? "Guess:" : "Trigger:"}
                  </div>
                  <input
                    type="range"
                    min="0"
                    max="1"
                    step="0.01"
                    value={clampEventPosition(marker.position)}
                    disabled={line.locked || Boolean(marker.positionExpression)}
                    on:input={(event) => handleEventPositionInput(eventIdx, event)}
                    on:change={() => updateEventMarker(eventIdx, {}, true)}
                    class="w-32"
                  />
                  <div class="w-28">
                    <ExpressionInput
                      compact
                      {variables}
                      disabled={line.locked}
                      placeholder="0.5"
                      title="Trigger position 0-1, or a percentage above 1"
                      value={expressionDisplayValue(
                        marker.positionExpression,
                        clampEventPosition(marker.position),
                      )}
                      onInput={(raw) =>
                        setEventExpression(eventIdx, "positionExpression", raw)}
                      onCommit={() => updateEventMarker(eventIdx, {}, true)}
                    />
                  </div>

                  {#if eventTriggerType(marker) === "pose"}
                    <div
                      class="font-extralight"
                      use:scrubbable={{
                        value: expressionDisplayValue(
                          marker.poseXExpression,
                          eventPoseX(marker),
                        ),
                        disabled: line.locked,
                        onInput: (raw) =>
                          setEventExpression(eventIdx, "poseXExpression", raw),
                        onCommit: () => updateEventMarker(eventIdx, {}, true),
                      }}
                    >X:</div>
                    <div class="w-24">
                      <ExpressionInput
                        compact
                        {variables}
                        disabled={line.locked}
                        title="Pose X (literal or expression)"
                        value={expressionDisplayValue(
                          marker.poseXExpression,
                          eventPoseX(marker),
                        )}
                        onInput={(raw) =>
                          setEventExpression(eventIdx, "poseXExpression", raw)}
                        onCommit={() => updateEventMarker(eventIdx, {}, true)}
                      />
                    </div>
                    <div
                      class="font-extralight"
                      use:scrubbable={{
                        value: expressionDisplayValue(
                          marker.poseYExpression,
                          eventPoseY(marker),
                        ),
                        disabled: line.locked,
                        onInput: (raw) =>
                          setEventExpression(eventIdx, "poseYExpression", raw),
                        onCommit: () => updateEventMarker(eventIdx, {}, true),
                      }}
                    >Y:</div>
                    <div class="w-24">
                      <ExpressionInput
                        compact
                        {variables}
                        disabled={line.locked}
                        title="Pose Y (literal or expression)"
                        value={expressionDisplayValue(
                          marker.poseYExpression,
                          eventPoseY(marker),
                        )}
                        onInput={(raw) =>
                          setEventExpression(eventIdx, "poseYExpression", raw)}
                        onCommit={() => updateEventMarker(eventIdx, {}, true)}
                      />
                    </div>
                  {/if}
                {/if}

                <div class="font-extralight">Duration ms:</div>
                <div class="w-28">
                  <ExpressionInput
                    compact
                    {variables}
                    disabled={line.locked}
                    placeholder="0"
                    title="0 keeps the event active until auto end"
                    value={expressionDisplayValue(
                      marker.durationExpression,
                      clampEventDuration(marker.durationMs),
                    )}
                    onInput={(raw) =>
                      setEventExpression(eventIdx, "durationExpression", raw)}
                    onCommit={() => updateEventMarker(eventIdx, {}, true)}
                  />
                </div>

                <div class="font-extralight">If:</div>
                <div class="w-28">
                  <ExpressionInput
                    compact
                    kind="boolean"
                    {variables}
                    disabled={line.locked}
                    placeholder="always"
                    title="Boolean expression; when false this event is skipped"
                    value={marker.enabledExpression || ""}
                    onInput={(raw) =>
                      setEventExpression(eventIdx, "enabledExpression", raw)}
                    onCommit={() => updateEventMarker(eventIdx, {}, true)}
                  />
                </div>

                <button
                  title={line.locked ? "Path locked" : "Remove parallel event"}
                  on:click={() => removeEventMarker(eventIdx)}
                  disabled={line.locked}
                  class="ml-auto p-1 rounded text-red-500 hover:text-red-600 disabled:opacity-40 disabled:cursor-not-allowed"
                >
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    stroke-width="2"
                    class="size-5"
                  >
                    <path stroke-linecap="round" stroke-linejoin="round" d="M6 12h12" />
                  </svg>
                </button>
              </div>
            {/each}
          </div>
        {/if}
      </div>
    </div>

    <ControlPointsSection
      bind:line
      lineIdx={idx}
      bind:collapsed={collapsedControlPoints}
      onAddControlPoint={onInsertAfter}
      {variables}
      {recordChange}
    />
  {/if}
</div>

<style>
  @keyframes rainbow-glow {
    0% {
      background-position: 0% 50%;
    }
    50% {
      background-position: 100% 50%;
    }
    100% {
      background-position: 0% 50%;
    }
  }

</style>
