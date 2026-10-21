import { defineConfig } from 'vite';
// FREEZE=1 (film renders): no HMR, no file watching, so editing code mid-render cannot reload the page under
// tools/film/*.mjs. Pick a free port per server (npm run dev -- --port 5177) and pass port= to the tools.
const freeze = !!process.env.FREEZE;
export default defineConfig({
  server: { port: 5173, strictPort: true, ...(freeze ? { hmr: false, watch: null } : {}) },
  worker: { format: 'es' },
  build: { target: 'es2022', chunkSizeWarningLimit: 4000 },
});
