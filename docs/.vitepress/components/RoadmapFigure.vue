<script setup lang="ts">
import { computed, ref } from 'vue'

const props = defineProps<{ label: string; width?: number }>()
const zoom = ref(1)
const fitted = ref(true)
const diagramStyle = computed(() => ({
  width: fitted.value ? '100%' : `${(props.width ?? 900) * zoom.value}px`,
}))

function resize(delta: number) {
  fitted.value = false
  zoom.value = Math.min(2, Math.max(0.5, zoom.value + delta))
}

function fit() {
  zoom.value = 1
  fitted.value = true
}
</script>

<template>
  <figure class="roadmap-figure" :aria-label="label">
    <figcaption class="roadmap-toolbar">
      <span class="roadmap-label">{{ label }}</span>
      <div class="roadmap-controls" role="group" :aria-label="`${label}缩放`">
        <button type="button" :disabled="!fitted && zoom <= 0.5" aria-label="缩小路线图" @click="resize(-0.25)">−</button>
        <button type="button" aria-label="路线图适应宽度" @click="fit">适应宽度</button>
        <button type="button" :disabled="!fitted && zoom >= 2" aria-label="放大路线图" @click="resize(0.25)">＋</button>
      </div>
    </figcaption>
    <div class="roadmap-viewport" tabindex="0" role="region" :aria-label="`${label}，放大后可横向滚动`">
      <div class="roadmap-diagram" :style="diagramStyle"><slot /></div>
    </div>
  </figure>
</template>

<style scoped>
.roadmap-figure {
  margin: 24px 0;
  overflow: hidden;
  border: 1px solid var(--vp-c-divider);
  border-radius: 12px;
  background: var(--vp-c-bg);
}

.roadmap-toolbar {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  justify-content: space-between;
  gap: 8px;
  padding: 10px 14px;
  border-bottom: 1px solid var(--vp-c-divider);
  background: var(--vp-c-bg-soft);
  font-size: 13px;
}

.roadmap-label { font-weight: 600; }
.roadmap-controls { display: flex; gap: 4px; }
.roadmap-controls button {
  min-width: 30px;
  min-height: 30px;
  padding: 2px 8px;
  border: 1px solid var(--vp-c-divider);
  border-radius: 6px;
  background: var(--vp-c-bg);
  color: var(--vp-c-text-1);
  cursor: pointer;
}
.roadmap-controls button:hover:not(:disabled) { border-color: var(--vp-c-brand-1); }
.roadmap-controls button:disabled { opacity: 0.4; cursor: default; }
.roadmap-controls button:focus-visible,
.roadmap-viewport:focus-visible { outline: 2px solid var(--vp-c-brand-1); outline-offset: -2px; }
.roadmap-viewport { overflow: auto; padding: 16px; }
.roadmap-diagram { margin: 0 auto; }
.roadmap-diagram :deep(.mermaid) { margin: 0; }
.roadmap-diagram :deep(svg) { display: block; width: 100%; max-width: none !important; height: auto; }
/* Keep HTML labels consistent with Mermaid's measurements outside .vp-doc. */
.roadmap-diagram :deep(.mermaid p) {
  margin: 0;
  line-height: inherit;
}
</style>
